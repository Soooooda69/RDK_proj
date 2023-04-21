%test ur5RRcontrol
clc
clear
ur5=ur5_interface();
q=[pi/3; pi/4; -pi/3; -pi/3; pi/3; -pi/4];
ur5.move_joints(q, 15)
pause(15)
q = [-pi/3; pi/3; 0; -pi/2; pi/3; -pi/4];
gdesired=ur5FwdKin(q);
ur5RRcontrol( gdesired, 1, ur5 )
pause(20)