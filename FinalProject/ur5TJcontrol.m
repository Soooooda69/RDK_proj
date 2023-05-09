function TJ = ur5TJcontrol( gdesired, K, ur5 )

    T = 0.4;
    q = ur5.get_current_joints();
    % Get the current pose
    g = ur5FwdKin(q);
    xi = getXi(gdesired \ g);
    v = xi(1:3);
    w = xi(4:6);
    m = 20;
    %Show desired frame
    Frame_desired = tf_frame('base_link','Frame_desired',gdesired);
    while or(norm(v) >0.003, norm(w) > pi/180)%Condition of stop
        qk = ur5.get_current_joints();
        gk = ur5FwdKin(qk);
        J = ur5BodyJacobian(qk);
        xi = getXi(gdesired \ gk);
        v = xi(1:3);
        w = xi(4:6);
        dq = K*T*(J'*xi);
        if v > 0.5
            xi = xi/4;
        end
        if norm(xi)<0.6
            T = (xi'*J*J'*xi)/norm(J*J'*xi)
        end

        if 0.0001<norm(dq)&&norm(dq)<0.02 
            dq = dq*0.02/norm(K*T*(J'*xi));
            m = 5
        elseif norm(dq)<0.0001
            dq = dq*0.001/norm(K*T*(J'*xi));  
            m = 3
        end
        qk = qk - dq; 
% Test manipulability    
        if abs(manipulability(J,'sigmamin')) < 0.0001
            finalerr = -1;
            return
        end
        t = max(abs(dq)/(ur5.speed_limit*pi))*m;
        ur5.move_joints(qk,t);
        pause(t);

    end  
end


   

