%test ur5RRcontrol
clc
clear
ur5=ur5_interface();
ur5.move_joints(ur5.home, 3);
pause(4);

q1=[pi/8; pi/8; pi/8; pi/8; pi/8; pi/8];
ur5.move_joints(q1+ur5.home, 3);
pause(4);
q2 = [pi/6; pi/7; pi/7; pi/8; pi/6; pi/8];
gdesired=ur5FwdKin(q2);
ur5RRcontrol(gdesired, 0.1, ur5);
pause(20);