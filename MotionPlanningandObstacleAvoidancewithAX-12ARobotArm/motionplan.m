function qref = motionplan(q0, qf, t1, t2, myrobot, obs, tol)
%Takes a start configuration q0, end configuration qf, start time t1, end
%time t2, robot structure myrobot, cell of obstacle structures obs, and
%tolerance tol. Returns a set of joint angles that move the robot from q0
%to qf withing tol in the given times while avoiding obstacles
    i = 1;
    a = 0.01;
    q = q0';
    qf = qf';
    while(norm(q(1:3,i) - qf(1:3)) >= tol) && i < 10000                 %loop until current configuration is within tol of end configuration
        tau_att = att(q(:,i), qf, myrobot)';                %calculate the joint torques from attractive potential
        tau_rep = 0;
        for j = 1:numel(obs)
            tau_rep = tau_rep + rep(q(:,i), myrobot, obs{j})';      %calculate the joint torques from repulsive potential for each obstacle
        end
        q(:,i+1) = q(:,i) + a*1*tau_att + a*tau_rep;          %determine the next configuration
        i = i + 1;
        
        disp(norm(q(1:3,i) - qf(1:3)))
    end

    q(4,:) = linspace(q0(4),qf(4), numel(q(4,:)));          %q4 is not affected by the potentials, so linearly transitions from start to end configuration
    t = linspace(t1, t2, size(q,2));
    qref = spline(t,q);                                     %use spline to create continuous coniguration change            
end