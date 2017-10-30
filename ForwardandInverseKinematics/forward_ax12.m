function H = forward_ax12(myrobot, q)
%Takes 1x4 joint angle vector q and robot structure and returns homogeneous
%linear transformation matrix H giving end effector position and orientation
%with respect to frame 0 

    H = eye(4);                                         %initialize as identity matrix 
    for i = 1:numel(myrobot.link)
        H = H * getH(myrobot.link{i}, q(i));            %multiply H1*H2*H3*H4 to get H
    end
end