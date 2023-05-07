function mu = manipulability(J, measure)
% J: a 6 × 6 matrix.
% measure: a single string argument that can only be one of ‘sigmamin’, 
% 'detjac', or 'invcond . Defines which manipulability measure is used.
% mu: The corresponding measure of manipulability.

e = sqrt(eig(J'*J));
if strcmp("sigmamin", measure) == 1
    mu = min(e);
end
if strcmp("invcond", measure) == 1
    mu = min(e) / max(e);
end
if strcmp("detjac", measure) == 1
    mu = det(J);
end
end