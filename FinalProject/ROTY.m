function R = ROTY(roll)
% ROTX: Generates a 3x3 rotation matrix for a given roll angle.
% Inputs:
%   - roll: The roll angle (in radians).
% Outputs:
%   - R: The 3x3 rotation matrix.

% Compute the sin and cosine of the roll angle
c = cos(roll);
s = sin(roll);

% Define the rotation matrix
R = [c 0 s; 0 1 0; -s 0 c];
end