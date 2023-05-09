function xi = getXi(g)
%Take a homogenous transformation matrix and extract the unscaled twist.
%Input: g: a homogeneous transformation.
%Output: xi: the (un-normalized) twist in 6 × 1 vector or twist coordinate.
    R = g(1:3, 1:3);
    p  =g(1:3, 4);
    theta = acos((trace(R) - 1)/2);
    if theta == 0 
          w = zeros(3,1);
          v = p/norm(g(1:3,4));
          xi = [v;w];
          return;
     end
    w = INVSKEW3(R - R')/(2*sin(theta));
    tmp = (eye(3) - R)*SKEW3(w) + w*w.'*theta;
    v = inv(tmp)*p;
    xi = [v;w]*theta;