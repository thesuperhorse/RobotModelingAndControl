function H = getH(link, joint_theta)
%Takes link variable and a joint angle and returns the homogenous linear
%transformation matrix H giving position and orientation of frame i with
%respect to frame i-1. The link variable comes from the robot and the
%joint_theta supplements the link information since the theta parameter in
%the DH table of the robot is a placeholder 0


    A = link.A;                             %DH variable a_i for link i
    alpha = link.alpha;                     %DH variable alpha_i for link i
    theta = joint_theta;                    %DH variable theta_i for link i
    D = link.D;                             %DH variable d_i for link i
    
    
    H = [
        cos(theta) -sin(theta)*cos(alpha) sin(theta)*sin(alpha) A*cos(theta);
        sin(theta) cos(theta)*cos(alpha) -cos(theta)*sin(alpha) A*sin(theta);
        0 sin(alpha) cos(alpha) D;
        0 0 0 1];                           %formula for H from DH parameters   

end