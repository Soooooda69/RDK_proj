function T_inv = FINV(T)
% FINV computes the matrix inverse of a 4x4 homogeneous transformation T

% Extract the rotation matrix and translation vector from T
R = T(1:3, 1:3);
t = T(1:3, 4);

% Compute the inverse of the rotation matrix
R_inv = R';

% Compute the inverse of the translation vector
t_inv = -R_inv * t;

% Construct the inverse transformation matrix
T_inv = [R_inv, t_inv; 0, 0, 0, 1];
end
