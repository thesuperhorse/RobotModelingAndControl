function tau = att(q, qf, myrobot)
%Takes curren configuration q, target configuration qf, and robot structure myrobot. Returns the normalized joint torques resulting from attractive forces.
    H_start = forward(q, myrobot);                                  
    H_end = forward(qf, myrobot);                                   
    O_start  = H_start(1:3, 4);
    O_end  = H_end(1:3, 4);
    
    zeta = 1;
    d = 9000;                                                   %arbitrary large boundary
    norm_d =  norm(O_end - O_start, 2);
    
    well_close = (0.5) * zeta * norm_d^2;
    well_far = d * zeta * norm_d - (0.5) * zeta * d^2;
    
    
    H01 = getH(myrobot.link{1}, q(1));                          %find the homogeneous transformation matrices from DH table for curent configuration
    H12 = getH(myrobot.link{2}, q(2));
    H23 = getH(myrobot.link{3}, q(3));
    H34 = getH(myrobot.link{4}, q(4));
    H45 = getH(myrobot.link{5}, q(5));
    H56 = getH(myrobot.link{6}, q(6));
    
    H02 = H01*H12;                                              %calculate homogeneous transformation matrices to relate each frame to base frame for current configuration
    H03 = H02*H23;
    H04 = H03*H34;
    H05 = H04*H45;
    H06 = H05*H56;


    H01f = getH(myrobot.link{1}, qf(1));                        %find the homogeneous transformation matrices from DH table for final configuration                    
    H12f = getH(myrobot.link{2}, qf(2));
    H23f = getH(myrobot.link{3}, qf(3));
    H34f = getH(myrobot.link{4}, qf(4));
    H45f = getH(myrobot.link{5}, qf(5));
    H56f = getH(myrobot.link{6}, qf(6));
    
    H02f = H01f*H12f;                                           %calculate homogeneous transformation matrices to relate each frame to base frame for final configuration
    H03f = H02f*H23f;
    H04f = H03f*H34f;
    H05f = H04f*H45f;
    H06f = H05f*H56f;
    
    origin_start = {H01(1:3,4), H02(1:3,4), H03(1:3,4), H04(1:3,4), H05(1:3,4), H06(1:3,4)};
    origin_final = {H01f(1:3,4), H02f(1:3,4), H03f(1:3,4), H04f(1:3,4), H05f(1:3,4), H06f(1:3,4)};
       
    
    F_vec = [];
    for j = 1:6                                                 %loop through joints to calculate attractive forces
        if( norm_d <= d)                                        %if current origin within boundary, calculate attractive force normally
            F_att = -zeta * (origin_start{j} - origin_final{j});
        else                                                    %otherwise, saturate the force
            F_att = - d * zeta * (origin_start{j} - origin_final{j})/norm_d;
        end
        F_vec(:,j) = F_att;
    end
    
    

    tau = 0;
    for k = 1:6                                                 %loop through the joints
        J_joint = getJ(myrobot, q, k);                          %calculate the linear velocity Jacobian for current joint
        tau = tau + J_joint'*F_vec(:,k);                        %calculate the corresponding joint torques and add to previous torques
    end
    
    tau = tau'/norm(tau);                                        %normalize joint torques
end 
    
    
    
    
    
    
    
    
    