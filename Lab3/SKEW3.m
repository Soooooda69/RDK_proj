function S = SKEW3(x)
% SKEW3: Generates a 3x3 skew-symmetric matrix for a given 3x1 vector x.
% Inputs:
%   - x: A 3x1 vector [x1, x2, x3].
% Outputs:
%   - S: The 3x3 skew-symmetric matrix.

% Extract the individual components of x
x1 = x(1);
x2 = x(2);
x3 = x(3);

% Define the skew-symmetric matrix
S = [0, -x3, x2;     x3, 0, -x1;     -x2, x1, 0];
end
