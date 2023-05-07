function R = EXPCR(xi,theta)
% Return the 3 × 3 rotation matrix which represents a rigid-body rotation
    w = xi(4:6);
    if theta == 0
        R = eye(3);
        return;
    end
    R = eye(3)+ SKEW3(w)*sin(theta) + SKEW3(w)* SKEW3(w)*(1-cos(theta));
end