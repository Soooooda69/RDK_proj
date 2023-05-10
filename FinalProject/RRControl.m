function [] = RRControl(ur5, joint_list, startpose, pose1, pose2, endpose, startpos, pos1, pos2, endpos, joint_offset)
PlanTraj(startpos, pos1, pos2, endpos);
% Move to initial positioln
startjoint = joint_list(:,1);
ur5.move_joints(startjoint, 6);
pause(6);

disp('**************Error analysis for the start location**************')
[d_SO3, d_R3] = SE3errors(ur5.get_current_transformation('base_link', 'tool0'), ...
    startpose);

% first segment
cur_g = startpose;
cur_p = cur_g(1:3,4);
cur_j = startjoint - joint_offset;
dt = 0.1;
v = Xi(pinv(pose1)*cur_g);
while norm(pos1 - cur_p) > 0.005
    j = ur5BodyJacobian(cur_j);
    next_j = cur_j - dt * pinv(j) * v;
    %next_j = cur_j - dt * j' * v;
    ur5.move_joints(next_j + joint_offset,0.5);
    pause(1);
    cur_j = ur5.get_current_joints() - joint_offset;
    cur_g = ur5.get_current_transformation('base_link', 'tool0');
    cur_p = cur_g(1:3,4);
    scatter3(cur_p(1), cur_p(2), cur_p(3));
    hold on
    v = Xi(pinv(pose1)*cur_g);
end
disp("first segment is finished")

% second segment
v = Xi(pinv(pose2)*cur_g);
dt = 0.1;
while norm(cur_p - pos2) >= 0.003
    j = ur5BodyJacobian(cur_j);
    next_j = cur_j - dt * pinv(j) * v;
    %next_j = cur_j - dt * j' * v;
    ur5.move_joints(next_j + joint_offset,0.5);
    pause(0.5);
    cur_j = ur5.get_current_joints()- joint_offset;
    cur_g = ur5.get_current_transformation('base_link', 'tool0');
    cur_p = cur_g(1:3,4);
    scatter3(cur_p(1), cur_p(2), cur_p(3));
    hold on
    v = Xi(pinv(pose2)*cur_g);
end
disp("second segment is finished")

% third segment
v = Xi(pinv(endpose)*cur_g);
dt = 0.1;
while norm(cur_p - endpos) >= 0.003
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
disp('**************Error analysis for the target location**************')
[d_SO3, d_R3] = SE3errors(ur5.get_current_transformation('base_link', 'tool0'), ...
    endpose);
end