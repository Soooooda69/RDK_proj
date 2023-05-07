%% Start and initialize
clear;
clc;
rosshutdown;
ur5 = ur5_interface();
joint_offset = [-pi 0 0 0 0 0]';
% Set UR5 back to home pose
joints = [0 0 0 0 0 0]';
ur5.move_joints(joints,3);

%transformation from keating base to {S}, g_0->S
g_baseK_S = [ROTZ(0) [0 0 0.0892]'; 0 0 0 1];  
%-90 degree rotation around z and up x 0.0892 
tf_frame('base_link','S',g_baseK_S);
pause(0.5);

%transformation from {T} to keating tool, g_T->6
g_T_toolK = [ROTX(-pi/2)*ROTY(pi/2) [0 0 0]'; 0 0 0 1];
%-90 around x and 90 around y
tf_frame('tool0','T',inv(g_T_toolK));
pause(0.5);

% Tool frame to the pen tip 
g_T_tip = [eye(3), [0.12228, 0, 0.049]'; 0,0,0,1];

% g_S_tip = g_S_T * g_T_tip
g_S_T = ur5FwdKin(joints);
g_S_tip = g_S_T * g_T_tip;
tip_frame = tf_frame('S','tip',g_S_tip);
pause(0.5);

%% Place the UR5 end effector to near plane
% TODO: Safety check is needed
% % Should get the practical position as the initial pose
% % 
% % gst1 = ur5.get_current_transformation('S', 'tip');
gst1 = [ROTZ(-pi/4)*ROTY(pi/5), [0.3, -0.4, 0.22]'; 0,0,0,1];
tip_frame.move_frame('T', g_T_tip);
thetas = ur5InvKin(gst1);
ur5.move_joints(thetas(:,3)-joint_offset,5);
pause(5);

%% Get the start and end
clc;
% Show an ironman face as the user interface
f1 = figure;
set(f1, 'WindowStyle', 'modal');
imshow('ironman.jpg');

disp('*************Press R to record current location*************');
% Save g_S_tip
tip_pose_list = {};
i = 0;
while (true)
    pause(0.1);
    keyPress = get(gcf,'CurrentCharacter');
    % Press "R" to record the current location 
    if strcmpi(keyPress,'r')
        % Reset the current pressed key
        set(gcf, 'CurrentCharacter', '1');
        g_S_T = ur5.get_current_transformation('S', 'T');  
        thetas = ur5InvKin(g_S_T);
        disp('The current joints configurations:');
        disp(thetas(:,1));
        pause(0.1);
        tip_pose_list{end+1} = g_S_T * g_T_tip;
        i = i+1;
        fprintf('%dth current location recorded!\n', i)
        disp('*************Press Q to finish recording*************');
        % Press "Q" to exit
    elseif strcmpi(keyPress,'q')
        close(f1);
        break
    end
end

%% Plan the trajectory

% Test config switch
config='J';

% Plan pose list g_S_T and joints config list
plan_pose_list = {};
plan_joints_list = {};

start_pose = tip_pose_list{1};
mid_pose = tip_pose_list{2};
end_pose = tip_pose_list{end};
% disp(start_pose);
% disp(mid_pose);
% disp(end_pose);
p1 = start_pose(1:3, 4)';
p2 = mid_pose(1:3, 4)';
p3 = end_pose(1:3, 4)';

% Trajectory width
width = 0.05;

% Plan the trajectory for the move
lines = PlanTraj(p1, p2, p3, width);

if (config=='JT')
    len = 1;
else
    len = 3;
end

% Draw the planned trajectory
for i=1:len
plot3(lines{i}(1,:), lines{i}(2,:), lines{i}(3,:), 'r-', 'LineWidth', 1);
hold on;
end
% plot3(lines{1}(1,:), lines{1}(2,:), lines{1}(3,:), 'r-', 'LineWidth', 1);
% hold on;
% plot3(lines{2}(1,:), lines{2}(2,:), lines{2}(3,:), 'g-', 'LineWidth', 1);
% plot3(lines{3}(1,:), lines{3}(2,:), lines{3}(3,:), 'b-', 'LineWidth', 1);

% The start_pose is g_S_tip
tmp_pose = start_pose;

for i=1:length(lines)
    for j=1:length(lines{1})
        tmp_pose(1:3, 4)=lines{i}(:,j);
        plan_pose_list{end+1}=tmp_pose * inv(g_T_tip);
        thetas = ur5InvKin(tmp_pose * inv(g_T_tip));
        plan_joints_list{end+1}=thetas(:,3);
    end
end

%% IK solution
% Set UR5 to the start pose
ur5.move_joints(plan_joints_list{1}-joint_offset,3);
pause(3);

% Start error
disp('**************Error analysis for the start location**************')
[d_SO3, d_R3] = SE3errors(ur5.get_current_transformation('S', 'tip'), ...
    plan_pose_list{1});

% Move the UR5 with IK solution
for i=1:len
plot3(lines{i}(1,:), lines{i}(2,:), lines{i}(3,:), 'r-', 'LineWidth', 1);
hold on;
end

for i=2:length(plan_joints_list)
    fprintf('%dth moving\n', i);
    ur5.move_joints(plan_joints_list{i}-joint_offset,0.1);
    pause(0.1);
    tmp_curr = ur5.get_current_transformation('S', 'tip');
    p = tmp_curr(1:3, 4);
    scatter3(p(1), p(2), p(3),'MarkerFaceColor',[0 .75 .75]);
    hold on;
%     pause(0.1);
end

% Target error
disp('**************Error analysis for the target location**************')
[d_SO3, d_R3] = SE3errors(ur5.get_current_transformation('S', 'tip'), ...
    plan_pose_list{end});

