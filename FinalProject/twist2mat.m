function homoMat = twist2mat(xi, theta)
% twist2mat convert a twist to the 4x4 homogeneous transformation matrix. 
    v = xi(1:3);
    w = xi(4:6);
    R = EXPCR(xi, theta);
    p = (eye(3) - R)*cross(w, v)+w*w'*v*theta;
    homoMat = [R, p; 0,0,0,1];
end

