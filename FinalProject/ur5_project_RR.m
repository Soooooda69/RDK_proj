%% Start and initialize
clear;
clc;
rosshutdown;
ur5 = ur5_interface();


% Set UR5 back to home pose
joints = [0 0 0 0 0 0]';
joint_offset = [0 -pi/2 0 -pi/2 0 0]';
ur5.move_joints(joints + joint_offset,10);

% %transformation from keating base to {S}, g_0->S
% g_baseK_S = [ROTZ(0) [0 0 0.0892]'; 0 0 0 1];  
% %-90 degree rotation around z and up x 0.0892 
% tf_frame('base_link','S',g_baseK_S);
% pause(0.5);
% 
% %transformation from {T} to keating tool, g_T->6
% g_T_toolK = [ROTX(-pi/2)*ROTY(pi/2) [0 0 0]'; 0 0 0 1];
% %-90 around x and 90 around y
% tf_frame('tool0','T',inv(g_T_toolK));
% pause(0.5);
% 
% % Tool frame to the pen tip 
% g_T_tip = [eye(3), [0.12, 0, 0.049]'; 0,0,0,1];

g_tool_tip =  [eye(3) [0, -0.05, 0.125]'; 0 0 0 1];

% g_S_tip = g_S_T * g_T_tip
% g_S_T = ur5FwdKin(joints);
% g_S_tip = g_S_T * g_T_tip;
tf_frame('tool0','tip',g_tool_tip);
pause(1);

%% Get the start and end
%clc;
% Show an ironman face as the user interface
f1 = figure;
set(f1, 'WindowStyle', 'modal');
imshow('ironman.jpg');

disp('*************Press R to record current location*************');
% Save g_S_tip
tip_pose_list = {};
joint_list = [];
i = 0;
while (true)
    pause(0.1);
    keyPress = get(gcf,'CurrentCharacter');
    % Press "R" to record the current location 
    if strcmpi(keyPress,'r')
        % Reset the current pressed key
        set(gcf, 'CurrentCharacter', '1');
        g_S_T = ur5.get_current_transformation('base_link', 'tool0');
        %thetas = ur5InvKin(g_S_T);
        joint_list = [joint_list, ur5.get_current_joints];
        pause(0.1);
        tip_pose_list{end+1} = g_S_T;
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

% Trajectory width
width = 0.05;

%Plan the trajectory for the move
% lines = PlanTraj(p1, p2, p3, width);
% 
startpos = p1';
pos1 = startpos + [2/3 * (p3(1) - p1(1)) 0 0]';
% pos1 = startpos + [0 2/3 * (p3(2) - p1(2)) 0]';
pos2 = [pos1(1); p3(2); pos1(3)];
endpos = [p3(1);p3(2);p1(3)];

% [pos1, pos2] = getPath_init(startpos*1000, endpos*1000);

startpose = start_pose;

pose1 = start_pose;
pose1(1:3,4) = pos1;


pose2 = start_pose;
pose2(1:3,4) = pos2;


endpose = start_pose;
endpose(1:3,4) = endpos;

lines = PlanTraj(startpos, pos1, pos2, endpos);

%% Move to initial positioln

% two intermediate position
% startpos = lines{1}(:,1);
% pos1 = lines{1}(:,end);
% pos2 = lines{2}(:,end);
% endpos = lines{3}(:,end); 
startjoint = joint_list(:,1);

% move to the start position
ur5.move_joints(startjoint, 6);
scatter3(startpos(1), startpos(2), startpos(3));
scatter3(pos1(1), pos1(2), pos1(3));
scatter3(pos2(1), pos2(2), pos2(3));
scatter3(endpos(1), endpos(2), endpos(3));
hold on
pause(6);

%% move
% first segment
pose1 = end_pose;
pos1 = endpos;
cur_g = startpose;
cur_p = cur_g(1:3,4);
cur_j = startjoint - joint_offset;
dt = 0.1;
v = Xi(pinv(pose1)*cur_g);
while norm(pos1 - cur_p) > 0.005
%     j = ur5BodyJacobian(cur_j);
    %next_j = cur_j - dt * pinv(j) * v;
    next_j = cur_j - dt * j' * v;
    ur5.move_joints(next_j + joint_offset,0.5);
    pause(1);
    cur_j = ur5.get_current_joints() - joint_offset;
    cur_g = ur5.get_current_transformation('base_link', 'tool0');
    cur_p = cur_g(1:3,4)
    scatter3(cur_p(1), cur_p(2), cur_p(3));
    hold on
    v = Xi(pinv(pose1)*cur_g);
end

%%

% second segment
v = Xi(pinv(pose2)*cur_g);
dt = 0.5;
while norm(cur_p - pos2) >= 0.003
    j = ur5BodyJacobian(cur_j);
    %next_j = cur_j - dt * pinv(j) * v;
    next_j = cur_j - dt * j' * v;
    ur5.move_joints(next_j + joint_offset,0.5);
    pause(0.5);
    cur_j = ur5.get_current_joints()- joint_offset;
    cur_g = ur5.get_current_transformation('base_link', 'tool0');
    cur_p = cur_g(1:3,4);
    scatter3(cur_p(1), cur_p(2), cur_p(3));
    hold on
    v = Xi(pinv(pose2)*cur_g);
end

%%
% third segment
v = Xi(pinv(endpose)*cur_g);
dt = 0.5;
while norm(cur_p - endpos) >= 0.001
    j = ur5BodyJacobian(cur_j);
    next_j = cur_j - dt * pinv(j) * v;
    %next_j = cur_j - dt * j' * v;
    ur5.move_joints(next_j+ joint_offset,0.5);
    pause(0.5);
    cur_j = ur5.get_current_joints()-joint_offset;
    cur_g = ur5.get_current_transformation('base_link', 'tool0');
    cur_p = cur_g(1:3,4);
    scatter3(cur_p(1), cur_p(2), cur_p(3));
    hold on
    v = Xi(pinv(endpose)*cur_g);
end

