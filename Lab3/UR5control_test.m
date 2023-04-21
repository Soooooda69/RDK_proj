%test ur5RRcontrol
clc
clear
ur5=ur5_interface();
ur5.move_joints(ur5.home, 3);
pause(5);

q1=[pi/8; pi/8; pi/8; pi/8; pi/8; pi/8];
ur5.move_joints(q1+ur5.home, 5);
pause(6);
q2 = [pi/8; pi/7; pi/7; pi/8; pi/6; pi/6];
gdesired=ur5FwdKin(q2);
ur5RRcontrol(gdesired, 1, ur5);
pause(20);