function Jb = ur5BodyJacobian(theta_vec)
    % gst0 and the base twists

    l0 = 0.0892; l1 = 0.425; l2 = 0.392; l3 = 0.1093; l4 = 0.09475; l5 = 0.0825;
    e1 = [1; 0; 0]; e2 = [0; 1; 0]; e3 = [0; 0; 1];
    gst0 = [ROTX(-pi/2), [0; l3+l5; l0+l1+l2+l4]; 0 0 0 1];
    
    q = zeros(3,6);
    q(:,1) = [0; 0; 0];
    q(:,2) = [0; 0; 0];
    q(:,3) = [0; 0; 0+l1];
    q(:,4) = [0; 0; 0+l1+l2];
    q(:,5) = [l3; 0; 0];
    q(:,6) = [0; 0; 0+l1+l2+l4];
    
    w = zeros(3,6);
    w(:,1) = e3; w(:,2) = e1; w(:,3) = e1; w(:,4) = e1; w(:,5) = e3; w(:,6) = e1;
    
    xi = zeros(6);
    for i = 1:length(xi)
     xi(:,i) = [-cross(w(:,i),q(:,i));w(:,i)];
     xi(:,i) = adj([ROTZ(pi/2) [0 0 l0]'; 0 0 0 1])*xi(:,i); 
    end

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
