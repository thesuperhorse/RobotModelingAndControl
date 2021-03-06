function tau = rep(q, myrobot, obs)
%Takes a configuration q, robot structure myrobot, and obstacle structure obs. Returns the normalized joint torques resulting from repulsive forces.
    eta = 1;
    
    H01 = getH(myrobot.link{1}, q(1));                              %Find the homogeneous transformation matrices from DH table
    H12 = getH(myrobot.link{2}, q(2));
    H23 = getH(myrobot.link{3}, q(3));
    H34 = getH(myrobot.link{4}, q(4));

    H02 = H01*H12;                                                  %calculate the homogeneous transformation matrices to relate each frame to base frame
    H03 = H02*H23;
    H04 = H03*H34;
    
    origin_start = {H01(1:3,4), H02(1:3,4), H03(1:3,4), H04(1:3,4)};

    
    F_vec = [];
    for j = 1:4                                                     %loop through joints to calculate repulsive forces
       
        if strcmp(obs.type,'ground')%x-y plane force, assuming rho0 = 100

            rho = abs(origin_start{j}(3));
            grad_rho = [0;0;1];

        elseif strcmp(obs.type,'sph')                                    
            rho = norm(origin_start{j}-obs.c) - obs.R;              %formula for rho and grad_rho for sphere obstacle
            grad_rho = (origin_start{j}-obs.c)/norm(origin_start{j}-obs.c);
            
        else %cyl obstacle
            if origin_start{j}(3) <= obs.h
                rho = norm(origin_start{j}(1:2) - obs.c) - obs.R;       
                grad_rho = (origin_start{j}(1:2) - obs.c)/norm(origin_start{j}(1:2) - obs.c);
                grad_rho(3) = 0;
            elseif norm(origin_start{j}(1:2)-obs.c) <= obs.R
                rho = origin_start{j}(3) - obs.h;
                grad_rho = [0; 0; 1];
            else
                vec1 = (origin_start{j}(1:2) - obs.c)/norm((origin_start{j}(1:2) - obs.c))* (norm(origin_start{j}(1:2) - obs.c) - obs.R); 
                vec1(3) = 0;
                vec2 = (origin_start{j}(3)-obs.h)*[0; 0; 1];
                rho = norm(vec1 + vec2);
                grad_rho = (vec1 + vec2)/norm(vec1 + vec2);
%                 rho = norm([origin_start{j}(3)-obs.h, norm(origin_start{j}(1:2)-obs.c)-obs.R]);
            end
        end
        
        if rho > obs.rho0
            F_rep = [0; 0; 0];                                      %if point farther than rho0 from the obstacle, 0 repulsive force
        else
            F_rep = eta*(1/rho - 1/obs.rho0)/rho^2*grad_rho;        %otherwise, calculate repulsive force based on rho and grad rho
        end

        F_vec(:,j) = F_rep;
    end
    
    
    tau = 0;
    for k = 1:4                                                     %loop through the joints
        J_joint = getJ(myrobot, q, k);                              %calculate the linear velocity Jacobian for current joint
        tau = tau + J_joint'*F_vec(:,k);                            %calculate the corresponding joint torques and add to previous torques
    end
    
    if norm(tau) ~= 0
        tau = tau/norm(tau);                                        %normalize joint torques
    end
    tau = tau';
end