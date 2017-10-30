function Jlin = getJ(myrobot, q, n)
% Takes a robot structure myrobot, configuration q and joint number n. Returns the linear velocity Jacobian for joint n.
    H01 = getH(myrobot.link{1}, q(1));                              %Find the homogeneous transformation matrix from DH table
    H12 = getH(myrobot.link{2}, q(2));
    H23 = getH(myrobot.link{3}, q(3));
    H34 = getH(myrobot.link{4}, q(4));
    H45 = getH(myrobot.link{5}, q(5));
    H56 = getH(myrobot.link{6}, q(6));
    
    H02 = H01*H12;                                                  %calculate the homogeneous transformation matrix to relate each frame to base frame
    H03 = H02*H23;
    H04 = H03*H34;
    H05 = H04*H45;
    H06 = H05*H56;
    
    H_n = {H01, H02, H03, H04, H05, H06};
    
    
    Jlin(:,1) = cross([0 0 1]', H_n{n}(1:3,4));                                 %first column of the Jacobian
    Jlin(:,2) = (n>=2)*cross(H01(1:3,3), H_n{n}(1:3,4) - H01(1:3,4));           %second column of the Jacobian; if the current joint is before joint 2 then fill with 0s
    Jlin(:,3) = (n>=3)*cross(H02(1:3,3), H_n{n}(1:3,4) - H02(1:3,4));           %third column of the Jacobian; if the current joint is before joint 3 then fill with 0s
    Jlin(:,4) = (n>=4)*cross(H03(1:3,3), H_n{n}(1:3,4) - H03(1:3,4));           %fourth column of the Jacobian; if the current joint is before joint 4 then fill with 0s
    Jlin(:,5) = (n>=5)*cross(H04(1:3,3), H_n{n}(1:3,4) - H04(1:3,4));           %fifth column of the Jacobian; if the current joint is before joint 5 then fill with 0s
    Jlin(:,6) = (n>=6)*cross(H05(1:3,3), H_n{n}(1:3,4) - H05(1:3,4));           %sixth column of the Jacobian; if the current joint is before joint 6 then fill with 0s

end