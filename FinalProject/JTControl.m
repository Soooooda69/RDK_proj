function [] = JTControl(ur5, joint_list, startpose, endpose, startpos, endpos, joint_offset)
% Move to initial positioln
startjoint = joint_list(:,1);
ur5.move_joints(startjoint, 6);
pause(6);
scatter3(startpos(1), startpos(2), startpos(3));
scatter3(endpos(1), endpos(2), endpos(3));

disp('**************Error analysis for the start location**************')
[d_SO3, d_R3] = SE3errors(ur5.get_current_transformation('base_link', 'tool0'), ...
    startpose);
pose1 = endpose;
pos1 = endpos;
cur_g = startpose;
cur_p = cur_g(1:3,4);
cur_j = startjoint - joint_offset;
dt = 0.6;
v = Xi(pinv(pose1)*cur_g);
while norm(pos1 - cur_p) > 0.005
    j = ur5BodyJacobian(cur_j);
    %next_j = cur_j - dt * pinv(j) * v;
    next_j = cur_j - dt * j' * v;
    ur5.move_joints(next_j + joint_offset,0.5);
    pause(1);
    cur_j = ur5.get_current_joints() - joint_offset;
    cur_g = ur5.get_current_transformation('base_link', 'tool0');
    cur_p = cur_g(1:3,4);
    scatter3(cur_p(1), cur_p(2), cur_p(3));
    hold on
    v = Xi(pinv(pose1)*cur_g);
end
disp('**************Error analysis for the end location**************')
[d_SO3, d_R3] = SE3errors(ur5.get_current_transformation('base_link', 'tool0'), ...
    endpose);
end