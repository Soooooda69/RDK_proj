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

start_pose = tip_pose_list{1};
mid_pose = tip_pose_list{2};
end_pose = tip_pose_list{end};
disp(start_pose)
disp(mid_pose)
disp(end_pose)
p1 = start_pose(1:3, 4)';
p2 = mid_pose(1:3, 4)';
p3 = end_pose(1:3, 4)';


%% Plan the trajectory

% Test config switch
config='J';

% Plan pose list g_S_T and joints config list
plan_pose_list = {};
plan_joints_list = {};

% Test 3 points to define a plane
% p1 = [0, 0.05, 0];
% p2 = [0,0,0];
% p3 = [15, 0, 0];

start_pose = tip_pose_list{1};
mid1_pose = tip_pose_list{2};
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
%% Move

% two intermediate position
startpos = lines{1}{:,1};
pos1 = lines{1}{:,end};
pos2 = lines{2}{:,end};
endpos = lines{3}{:,end};
dt = 0.1;

% move to the start position
ur5.move_joints(start_pose - joint_offset, 3);
pause(3);

% first segment
v = (pos1 - startpos) ./ norm(pos1-startpos);
cur_j = start_pose;
cur_g = ur5FwdKin(cur_j);
cur_p = cur_g(1:3,4);

while norm(cur_p - pos1) >= 0.1
    j = ur5BodyJacobian(cur_j);
    next_j = cur_j + dt * inv(j) * v;
    ur5.move_joints(next_j-joint_offset,0.05);
    pause(0.05);
    cur_j = next_j;
    cur_g = ur5FwdKin(cur_j);
    cur_p = cur_g(1:3,4);
end

% second segment
v = (pos2 - pos1) ./ norm(pos2-pos1);
while norm(cur_p - pos2) >= 0.1
    j = ur5BodyJacobian(cur_j);
    next_j = cur_j + dt * inv(j) * v;
    ur5.move_joints(next_j-joint_offset,0.05);
    pause(0.05);
    cur_j = next_j;
    cur_g = ur5FwdKin(cur_j);
    cur_p = cur_g(1:3,4);
end

% second segment
v = (pos2 - pos1) ./ norm(pos2-pos1);
while norm(cur_p - pos2) >= 0.1
    j = ur5BodyJacobian(cur_j);
    next_j = cur_j + dt * inv(j) * v;
    ur5.move_joints(next_j-joint_offset,0.05);
    pause(0.05);
    cur_j = next_j;
    cur_g = ur5FwdKin(cur_j);
    cur_p = cur_g(1:3,4);
end

% third segment
v = (endpos - pos2) ./ norm(endpos-pos2);
while norm(cur_p - endpos) >= 0.1
    j = ur5BodyJacobian(cur_j);
    next_j = cur_j + dt * inv(j) * v;
    ur5.move_joints(next_j-joint_offset,0.05);
    pause(0.05);
    cur_j = next_j;
    cur_g = ur5FwdKin(cur_j);
    cur_p = cur_g(1:3,4);
end

