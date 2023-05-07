function [d_SO3, d_R3] = SE3errors(gd, g)
% SE3errors: Calculate the rotational and translational errors separately 
% between the desired pose and the actual pose.
% Inputs:
%   - gd: A 4x4 SE(3) transformation for desired pose.
%   - g: A 4x4 SE(3) transformation for the actual pose.
% Outputs:
%   - d_SO3: The rotational error.
%   - d_R3: The translational error.

R = g(1:3, 1:3);
Rd = gd(1:3, 1:3);
r = g(1:3, 4);
rd = gd(1:3 ,4);

d_SO3 = sqrt(trace((R - Rd) * (R - Rd)'));
d_R3 = norm(r - rd);

end