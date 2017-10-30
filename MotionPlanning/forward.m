function H = forward(joint, myrobot)
%Takes 6x1 joint angle vector and robot structure and returns homogeneous
%linear transformation matrix H giving end effector position and orientation
%with respect to frame 0 

    H = eye(4);                                             %initialize as identity matrix 
    for i = 1:6
        H = H * getH(myrobot.link{i}, joint(i));            %multiply H1*H2*H3*H4*H5*H6 to get H
    end
end