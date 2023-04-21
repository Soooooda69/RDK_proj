function finalerr =ur5RRcontrol(gdesired, K, ur5)
%Implementation of resolved rate control system
Interval=0.05;%time interval
min_mani=0.00005;%Minimum Manipulability
v_lim= 0.05;%threshold of v in m
w_lim= 15*pi/180;%threshold of w in rad

%Current pose
q=ur5.get_current_joints();%Joint angles
gst=ur5FwdKin(q);%Pose of tool frame
Xi=getXi(gdesired^(-1)*gst);
v=Xi(1:3);
w=Xi(4:6);

%Define target 
Frame_desired = tf_frame('base_link','Frame_desired',gdesired);
while (norm(v)>v_lim)||(norm(w)>w_lim)
qk=ur5.get_current_joints();
gk=ur5FwdKin(qk);
Jb=ur5BodyJacobian(qk);
Xi=getXi(gdesired^(-1)*gk);
v=Xi(1:3);
w=Xi(4:6);
qk=qk-K*Interval*(Jb^(-1)*Xi);
%Singularity check
check = manipulability(Jb,'sigmamin');
if (abs(check)<=min_mani)||(rank(Jb)~=6)
finalerr=-1;
disp(['Singularity, finalerr = ',num2str(finalerr)]);
return
end
%Speed limit
try
ur5.move_joints(qk, Interval);
catch err
if strcmp(err.message,'Velocity over speed limit, please increase time_interval')
K = K*2/3;
continue
end
end

pause(Interval)
end
gst_tool = ur5FwdKin(qk);
finalerr = norm(gst_tool(1:3,4));
disp(['Final error = ',num2str(finalerr)])
