function q = inverse(H, myrobot)
%Takes linear homogeneous transformation matrix relating end effector to
%base frame and a robot structure, and returns the joint variables
%q_1,...,q_6 corresponding to that position and orientation

    od = H(1:3,4);                              %desired position of end effector
    Rd = H(1:3,1:3);                            %desired orietnation of end effector
    oc = od- Rd*[0; 0; 20];                     %desired position of wrist center
    
    xc = oc(1);
    yc = oc(2);
    zc = oc(3);
    
    
    %solution to inverse position problem (apply formulas to find theta1,
    %theta2, theta3)
   
    theta1 = atan2(yc, xc) - atan2(23.65, real(sqrt(xc^2 + yc^2 - 23.65^2)));                                          
    D = (xc^2 + yc^2 - 23.65^2 + (zc - 76)^2 - 43.23^2 - 43.18^2)/(2*43.23*43.18);
    theta3 = atan2(D, real(sqrt(1-D^2)));
    theta2 = atan2(zc-76, real(sqrt(xc^2 + yc^2 - 23.65^2))) - atan2(-43.18*cos(theta3), 43.23 + 43.18*sin(theta3));
    
    
    
    H03 = eye(4);                                                   %initialize H03 as identity
    joint_angles = [theta1, theta2, theta3];
    for i = 1:3
        H03 = H03 * getH(myrobot.link{i}, joint_angles(i));         %calculate H03 as H01*H12*H23
    end
    
    R03 = H03(1:3,1:3);                                             %extract rotation matrix from H03
    A = R03'*Rd;                                                    %calculate R36 (=:A)
    

   %solution to inverse orientation problem (apply formulas to find theta4,
   %theta5, theta6
   theta4 = atan2(A(2,3), A(1,3));
   theta5 = atan2(real(sqrt(1-A(3,3)^2)), A(3,3));
   theta6 = atan2(A(3,2), -A(3,1));
    
    q = [theta1, theta2, theta3, theta4, theta5, theta6];
end