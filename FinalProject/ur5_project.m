%% Start and initialize
clear;
clc;
rosshutdown;
ur5 = ur5_interface();
% joint_offset = [pi/2 0 0 0 0 0]';
joint_offset = [0 pi/2 0 pi/2 0 0]';
% Set UR5 back to home pose
joints = [0 0 0 0 0 0]';
ur5.move_joints(joints - joint_offset,10);

%transformation from keating base to {S}, g_0->S
g_baseK_S = [ROTZ(pi/2) [0 0 0.0892]'; 0 0 0 1];  
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
tip_frame.move_frame('T', g_T_tip);
pause(0.5);

%% Get the start and end
% For this section, use the moving panel in simulation to find three points
% defining a plane which you want to draw the trajectory to. The first and 
% the last point define the start and target point respectively.

clc;
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
%         thetas = ur5InvKin(g_S_T);
        joint_list = [joint_list, ur5.get_current_joints];
        disp('The current ee location:');
        disp(g_S_T(1:3, 4));
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

%% Plan the trajectory

start_pose = tip_pose_list{1};
mid_pose = tip_pose_list{2};
end_pose = tip_pose_list{end};
p1 = start_pose(1:3, 4)';
p2 = mid_pose(1:3, 4)';
p3 = end_pose(1:3, 4)';

startpos = p1';
pos1 = startpos + [2/3 * (p3(1) - p1(1)) 0 0]';
pos2 = [pos1(1); p3(2); pos1(3)];
endpos = [p3(1);p3(2);p1(3)];
startpose = start_pose;
pose1 = start_pose;
pose1(1:3,4) = pos1;
pose2 = start_pose;
pose2(1:3,4) = pos2;
endpose = start_pose;
endpose(1:3,4) = endpos;
%% Choose the method of control
% m = "inv" / "RR" / "JT"
m = "JT";
%% Move the robot
if strcmp(m,'inv') == 1
    invControl(ur5, joint_list, startpos, pos1, pos2, endpos, joint_offset)
end
if strcmp(m, 'RR') == 1
    RRControl(ur5, joint_list, startpose, pose1, pose2, endpose, startpos, pos1, pos2, endpos, -joint_offset)
end
if strcmp(m, 'JT') == 1
    JTControl(ur5, joint_list, startpose, endpose, startpos, endpos, -joint_offset)
end

%% extra credit
clc;
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
%         thetas = ur5InvKin(g_S_T);
        joint_list = [joint_list, ur5.get_current_joints];
        disp('The current ee location:');
        disp(g_S_T(1:3, 4));
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
%% plan and draw the figure
cur_joint = joint_list(:,1);
startpose = tip_pose_list{1};
startpos = startpose(1:3, 4);
path = load("Kirby.mat");
s = size(path.downsampled_track);
joints = [];
for i = 1 : 30 : s(1)
    dx = -path.downsampled_track(i,1) / 5000;
    dy = path.downsampled_track(i,2) / 5000;
    dz = 0;
    tmp_pose = startpose;
    tmp_pose(1:3,4) = startpos + [dx;dy;dz];
    thetas = ur5InvKin(tmp_pose);
    idx=-1;
    min=inf;
    for k=1:8
        tmp = norm(thetas(:,k)-joint_offset-cur_joint);
        if (min>tmp)
            min = tmp;
            idx = k;
        end
    end
    joints = [joints thetas(:,idx)];
end
for i=1:length(joints)
    ur5.move_joints(joints(:,i) - joint_offset,0.5);
    pause(0.5);
    tmp_curr = ur5.get_current_transformation('base_link','tool0');
    p = tmp_curr(1:3, 4);
    scatter3(p(1), p(2), p(3),'MarkerFaceColor',[0 .75 .75]);
    hold on;
end
    



