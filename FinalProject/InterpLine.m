function [x_values, y_values, z_values] = InterpLine(p1, p2, numPoints)
% Interpolate a line between p1 and p2.
% Inputs:
%   - p1: 1x3 start point.
%   - p2: 1x3 end point.
%   - numPoints: Number of points between.
% Outputs:
%   - [x_values, y_values, z_values]: Defined a line in 3d.


% Define a vector of values for t
t_values = linspace(0, 1, numPoints);

% Calculate the coordinates of points along the line
x_values = p1(1) + t_values * (p2(1) - p1(1));
y_values = p1(2) + t_values * (p2(2) - p1(2));
z_values = p1(3) + t_values * (p2(3) - p1(3));