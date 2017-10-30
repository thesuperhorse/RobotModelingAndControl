function q = inverse_ax12(myrobot, X)
%Takes linear homogeneous transformation matrix relating end effector to
%base frame and a robot structure, and returns the joint variables
%q_1,...,q_6 corresponding to that position and orientation
  
    xc = X(1);
    yc = X(2);
    zc = X(3);
    
    link1 = myrobot.link{1};
    link2 = myrobot.link{2};
    link3 = myrobot.link{3};
    link4 = myrobot.link{4};
    
    
    %solution to inverse position problem (apply formulas to find theta1,
    %theta2, theta3)
   
    theta1 = atan2(yc,xc) + atan2(link4.A, real(sqrt(xc^2+yc^2-link4.A^2)));                                
    
    delta = ( (zc - link1.D)^2 + (-link1.A + real(sqrt(xc^2 + yc^2-link4.A^2)))^2 - link2.A^2 - link4.D^2 - link3.A^2) / (-2*link2.A*real(sqrt(link4.D^2 + link3.A^2)));
    theta3 = acos(delta) - atan2(link3.A,link4.D) - pi/2;
    
    rho = atan2(real(sqrt(link4.D^2 + link3.A^2))*sin(pi/2 - theta3 - atan2(link3.A,link4.D)), link2.A + real(sqrt(link4.D^2 + link3.A^2))*cos(pi/2 - theta3 - atan2(link3.A,link4.D)));
    phi = atan2(zc - link1.D, -link1.A + real(sqrt(xc^2 + yc^2 - link4.A^2)));
    theta2 = rho + phi;
    
    q = [theta1, theta2, theta3, pi/2]';                %fix theta4 at pi/2
end