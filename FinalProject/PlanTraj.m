function lines = PlanTraj(p1, cornerpoint1, cornerpoint2, cornerpoint3)
% % Inputs:
% %   - p1: 1x3 start point.
% %   - p2: 1x3 intermidiate point.
% %   - p3: 1x3 target point.
% %   - hight_len: Length of the width.
% % Outputs:
% %   - lines: 3xN line trajectory.
lines = {};
% % Find the normal vector of the plane
% v1 = p1 - p2;
% v2 = p1 - p3;
% n = cross(v1, v2); 
% 
% % Find the length and direction of the diagonal
% d = norm(p3-p1);
% u = (p3-p1) / d;
% v = cross(n, u);
% 
% % solve the rectangle width vector and the height vector
% % hight_len = 0.05;
% syms wx wy wz hx hy hz
% w = [wx, wy, wz];
% eqns_w = [dot(u,w) == sqrt(d*d-hight_len*hight_len)/d, dot(n,w) == 0, norm(w)==1];
% sol_w = solve(eqns_w, w);
% w = double([sol_w.wx, sol_w.wy, sol_w.wz]);
% if (dot(v,w(1,:)) < 0)
%     w = w(1,:);
% else
%     w = w(2,:);
% end
% h = [hx, hy, hz];
% eqns_h = [dot(u,h) == hight_len/d, dot(n,h) == 0, norm(h)==1];
% sol_h = solve(eqns_h, h);
% h = double([sol_h.hx, sol_h.hy, sol_h.hz]);
% if (dot(v,h(1,:)) > 0)
%     h = h(1,:);
% else
%     h = h(2,:);
% end

interp_nums = 30;
% First trajectory
[x_values, y_values, z_values] = InterpLine(p1, cornerpoint1, interp_nums);
line1 = [x_values', y_values', z_values']';
lines{end+1} = line1;
plot3(x_values, y_values, z_values, 'r-', 'LineWidth', 1);
hold on;

% Second trajectory
[x_values, y_values, z_values] = InterpLine(cornerpoint1, cornerpoint2, interp_nums);
line2 = [x_values', y_values', z_values']';
lines{end+1} = line2;
plot3(x_values, y_values, z_values, 'b-', 'LineWidth', 1);

% Last trajectory
[x_values, y_values, z_values] = InterpLine(cornerpoint2, cornerpoint3, interp_nums);
line3 = [x_values', y_values', z_values']';
lines{end+1} = line3;
plot3(x_values, y_values, z_values, 'g-', 'LineWidth', 1);

% lines = [line1, line2, line3];
end