% Inverse Kinematics Test
clear;
clc;
rosshutdown;
ur5 = ur5_interface();

% Set UR5 back to home pose
ur5.move_joints([0 0 0 0 0 0]',3);
pause(3);
joint_offset = [-pi/2 -pi/2 0 -pi/2 0 0]';
% joint_offset = [-pi/2 0 0 0 0 0]';
joints = [pi/1 pi/5 -pi/5 pi/4 0 pi/2]';
g_S_T = ur5FwdKin(joints);

%transformation from keating base to {S}, g_0->S
g_baseK_S = [ROTZ(-pi/2) [0 0 0.0892]'; 0 0 0 1];  
%-90 degree rotation around z and up x 0.0892 
tf_frame('base_link','S',g_baseK_S);
pause(0.5);

%transformation from {T} to keating tool, g_T->6
g_T_toolK = [ROTX(-pi/2)*ROTY(pi/2) [0 0 0]'; 0 0 0 1];
%-90 around x and 90 around y
tf_frame('tool0','T',inv(g_T_toolK));
pause(0.5);

%transformation from keating base to keating tool
tf_frame('S','T2',g_S_T);
pause(0.5);

thetas = ur5InvKin(g_S_T);
disp(thetas-joint_offset)
ur5.move_joints(thetas(:,6)-joint_offset,3);
pause(5);

% errors
g = ur5.get_current_transformation('S','T');
pause(0.5);
[e_R, e_t] = SE3errors(g_S_T, g);
fprintf('Rotational error:%d\n',e_R);
fprintf('Translational error:%d\n',e_t);
pause(0.5);