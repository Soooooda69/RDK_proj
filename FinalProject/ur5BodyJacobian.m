function Jb = ur5BodyJacobian(theta_vec)

    % geometry of UR5 in meters
    L0 =0.0892; L1 = 0.425; L2 = 0.392; L3 = 0.1093; L4 = 0.09475; L5 = 0.0825;
    % gst0 and the base twists
    gst0 = [1,0,0,L3+L5;0,1,0,0;0,0,1,L0+L1+L2+L4;0,0,0,1];
    xi1=[0,0,0,0,0,1]';
    xi2=[0,L0,0,1,0,0]';
    xi3=[0,L0+L1,0,1,0,0]';
    xi4=[0,L0+L1+L2,0,1,0,0]';
    xi5=[0,-L3,0,0,0,1]';
    xi6=[0,L0+L1+L2+L4,0,1,0,0]';
    xi = [xi1, xi2, xi3, xi4, xi5, xi6];

    % transfer twists to SE(3) matrix form
    g1 = twist2mat(xi1, theta_vec(1));
    g2 = twist2mat(xi2, theta_vec(2));
    g3 = twist2mat(xi3, theta_vec(3));
    g4 = twist2mat(xi4, theta_vec(4));
    g5 = twist2mat(xi5, theta_vec(5));
    g6 = twist2mat(xi6, theta_vec(6));
    g = [g1, g2, g3, g4, g5, g6];

    % forward kinematics
    gstTheta = g1*g2*g3*g4*g5*g6*gst0;
    
    % body jacobian
    Jb = zeros(6,6);

    for i=1:6
        g_tmp = eye(4);
        for j=i:6
            g_tmp = g_tmp*twist2mat(xi(:, j), theta_vec(j));
        end
        G = g_tmp*gst0;
        R = G(1:3,1:3);
        p = G(1:3,4);
        pskew = [0, -p(3), p(2); p(3), 0, -p(1); -p(2), p(1), 0];
        adj_tmp = [R, pskew*R; zeros(3), R];
        if det(adj_tmp) == 0
            disp('The adjoint matrix is singular.');
            % Use pseudoinverse or regularization
            inv_adj_tmp = pinv(adj_tmp);
        else
            % Compute the inverse of the adjoint matrix
            inv_adj_tmp = inv(adj_tmp);
        end
        xi_tmp = inv_adj_tmp*xi(:, i);
        Jb(:, i) = xi_tmp;
        
    end 
end
