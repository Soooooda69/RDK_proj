function adj_m = adj(T)
    % Create adjoint matrix
    R = T(1:3, 1:3);
    p = T(1:3, 4);
    adj_m = [R, SKEW3(p)*R; zeros(3,3), R];