function j = jacobian_ur5(theta_vec, g_t_tip)
% geometry of UR5 in meters
L0 =0.0892; L1 = 0.425; L2 = 0.392; L3 = 0.1093; L4 = 0.09475; L5 = 0.0825;

syms t1 t2 t3 t4 t5 t6
g_baseK_S = [ROTZ(t1-pi/2) [0 0 L0]'; 0 0 0 1];  
g_S_U = [ROTY(t2), [0,L3,0]'; 0 0 0 1];
g_U_F = [ROTY(t3), [0,-L3,L1]'; 0 0 0 1];
g_F_W1 = [ROTY(t4), [0,0,L2]'; 0 0 0 1];
g_W1_W2 = [ROTZ(t5), [0,L3,0]'; 0 0 0 1];
g_W2_W3 = [ROTY(t6), [0,0,L4]'; 0 0 0 1];
g_W3_tool = [ROTX(-pi/2), [0,L5,0]'; 0 0 0 1];


% gst0 = [ROTZ(t1), [0,0,0]'; 0 0 0 1];
% gst1 = [ROTX(t2), [L3,0,0]'; 0 0 0 1];
% gst2 = [ROTX(t3), [0,0,L1]'; 0 0 0 1];
% gst3 = [ROTX(t4 + pi/2), [-L3,0,L2]'; 0 0 0 1];
% gst4 = [ROTZ(t5), [L3,0,0]'; 0 0 0 1];
% gst5 = [ROTX(t6), [0,0,L4]'; 0 0 0 1];
% gst6 = [ROTX(-pi/2), [L5,0,0]'; 0 0 0 1];

% gst0 = [ROTZ(t1), [L3,0,0]'; 0 0 0 1];
% gst1 = [ROTX(t2), [0,0,L1]'; 0 0 0 1];
% gst2 = [ROTX(t3), [-L3,0,L2]'; 0 0 0 1];
% gst3 = [ROTX(t4), [L3,0,0]'; 0 0 0 1];
% gst4 = [ROTZ(t5), [0,0,L4]'; 0 0 0 1];
% gst5 = [ROTX(t6), [L5,0,0]'; 0 0 0 1];
gstip = g_baseK_S*g_S_U*g_U_F*g_F_W1*g_W1_W2*g_W2_W3*g_W3_tool*g_t_tip;
% r1 = [cos(t1)*cos(t2+t3+t4+t5); cos(t2+t3+t4+t5)*sin(t1); -sin(t2+t3+t4+t5)];
% r2 = [-cos(t6)*sin(t1)+cos(t1)*sin(t2+t3+t4+t5)*sin(t6);...
%     1/2*(cos(t1-t6)+cos(t1+t6)+2*sin(t1)*sin(t2+t3+t4+t5)*sin(t6));...
%     cos(t2+t3+t4+t5)*sin(t6)];
% r3 = [cos(t1)*cos(t6)*sin(t2+t3+t4+t5)+sin(t1)*sin(t6);...
%     cos(t6)*sin(t1)*sin(t2+t3+t4+t5)-cos(t1)*sin(t6);...
%     cos(t2+t3+t4+t5)*cos(t6)];
% R = [r1 r2 r3];
% p = [L3 + cos(t1)*(-L3*cos(t2)+L3*cos(t2+t3)+L5*cos(t2+t3+t4+t5)+L2*sin(t2)+L4*sin(t2+t3+t4));...
%     sin(t1)*(-L3*cos(t2)+L3*cos(t2+t3)+L5*cos(t2+t3+t4+t5)+L2*sin(t2)+L4*sin(t2+t3+t4));...
%     L1+L2*cos(t2)+L4*cos(t2+t3+t4)+L3*sin(t2)-L3*sin(t2+t3)-L5*sin(t2+t3+t4+t5)];
% p = p + R*g_t_tip(1:3,4);
% j = jacobian(p,[t1,t2,t3,t4,t5,t6]);
% p = gstip(1:3,4);
p = gstip(1:3,4);
j = jacobian(p,[t1,t2,t3,t4,t5,t6]);
t1 = theta_vec(1);
t2 = theta_vec(2);
t3 = theta_vec(3);
t4 = theta_vec(4);
t5 = theta_vec(5);
t6 = theta_vec(6);
j = double(subs(j));
end