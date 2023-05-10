function [] = invControl(ur5, joint_list, startpos, pos1, pos2, endpos, joint_offset)
plan_pose_list = {};
plan_joints_list = {};
idx=-1;
min=inf;
cur_joint = joint_list(:,1);
lines = PlanTraj(startpos, pos1, pos2, endpos);
tmp_pose = start_pose;
for i=1:length(lines)
    for j=1:length(lines{1})
        tmp_pose(1:3, 4)=lines{i}(:,j);
        plan_pose_list{end+1}=tmp_pose;
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
        plan_joints_list{end+1}=thetas(:,idx);
    end
end

ur5.move_joints(joint_list(:,1),5);
pause(5);

% Start error
disp('**************Error analysis for the start location**************')
[d_SO3, d_R3] = SE3errors(ur5.get_current_transformation('base_link', 'tool0'), ...
    plan_pose_list{1});

for i=1:length(plan_joints_list)
    ur5.move_joints(plan_joints_list{i} - joint_offset,0.5);
    pause(0.5);
    tmp_curr = ur5.get_current_transformation('base_link','tool0');
    p = tmp_curr(1:3, 4);
    scatter3(p(1), p(2), p(3),'MarkerFaceColor',[0 .75 .75]);
    hold on;
%     pause(0.1);
end

% Target error
disp('**************Error analysis for the target location**************')
[d_SO3, d_R3] = SE3errors(ur5.get_current_transformation('base_link', 'tool0'), ...
    plan_pose_list{end});
end