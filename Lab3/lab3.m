
%% test ur5FwdKin
clear;
clc;
% init
ur5 = ur5_interface();
% set UR5 to home position
ur5.move_joints(ur5.home, 3);
pause(3);

% different positions to evaluate forward kinematics
q1 = [pi/3,pi/3,pi/3,pi/4,pi/4,pi/4]';
q2 = [0, pi/4, -pi/2, 0, pi/2, 0]';
q3 = [0, 0, -pi/5, 0, -pi/5, -pi/2]';
q4 = [pi/4, pi/5, pi/6, pi/7, pi/3, -pi/7]';
q5 = [0,0,0,0,0,0]';
q = [q1,q2,q3,q4, q5];

for i=1:5
    gstTheta = ur5FwdKin(q(:,i));
    % set the fwdKinToolFrame to new position
    fwdKinToolFrame = tf_frame('base_link','fwdKinToolFrame',eye(4));
    pause(0.5);
    fwdKinToolFrame.move_frame('base_link',gstTheta);
    % move UR5
    ur5.move_joints(q(:,i)+ur5.home, 3);
    pause(4);
    % calculate the difference by inv(gstReal)*gstTheta
    gstReal = ur5.get_current_transformation('base_link','ee_link');
    diff = FINV(gstReal)*gstTheta;
    disp('\tdifference between current position and theoretical value is')
    disp(diff)
end
%% Test ur5BodyJacobian
clear;
clc;
for j = 1:10
    q = [rand(1,6)*2*pi - pi]';
    g = ur5FwdKin(q);
    JB = ur5BodyJacobian(q);
    Japprox = zeros(6,6);       %matrix for jacobian approximation
    e = eye(6);
    eps = 1e-8;
    for i = 1:6
        ei = e(:,i);    %get the current basis vector
        dgdqi = 1/2/eps * ( ur5FwdKin(q + eps*ei) - ur5FwdKin(q - eps*ei) );
        xi_tmp = FINV(g)*dgdqi;
        Japprox(:,i) = [xi_tmp(1,4),xi_tmp(2,4),xi_tmp(3,4),...
            xi_tmp(3,2),-xi_tmp(3,1),xi_tmp(2,1)]';

    end
    %fprintf("case %d:\n",j)
    %JB
    %approx
    err = norm(JB - Japprox);
    fprintf('\terror between Jacobian and approximation is %d\n', err);
end
%% Test getXi
clc;
clear;
% generate random rotation matrix and translation vector
[R,~] = qr(randn(3));
p = randn(3,1);
g = [R,p;0,0,0,1];
disp('The randomly genreated transformation matrix:');
disp(g);
% extract unscaled twist with getXi
xi = getXi(g);
% exponentiate the xi_hat to get the transformation matrix
xi_hat = [SKEW3(xi(4:6)), xi(1:3); 0,0,0,0];
exp_xi = expm(xi_hat);
disp('The exponentiated xi_hat:');
disp(exp_xi);
disp('Difference between the initial transformation and the recovered one:');
disp(FINV(g)*exp_xi);

%% Test manipulability
q3_list = [-pi/4:0.01:pi/4];
m_m = [];
m_i = [];
m_d = [];
for q3 = q3_list
    J = ur5BodyJacobian([pi/2, pi/2, q3, pi/2, pi/2, 0]);
    m_m = [m_m manipulability(J, "sigmamin")];
    m_i = [m_i manipulability(J, "invcond")];
    m_d = [m_d manipulability(J, "detjac")];
end
figure
plot(q3_list, m_m);
xlabel("q3");
ylabel("manipulability");
figure
plot(q3_list, m_i);
xlabel("q3");
ylabel("manipulability");
figure
plot(q3_list, m_d);
xlabel("q3");
ylabel("manipulability");
