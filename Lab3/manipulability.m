function mu = manipulability(J, measure)
% J: a 6 × 6 matrix.
% measure: a single string argument that can only be one of ‘sigmamin’, 
% 'detjac', or 'invcond . Defines which manipulability measure is used.
% mu: The corresponding measure of manipulability.

