%% Part 4.1
m = manipulator(4, 0);                  %initialize manipulator

setangles(m, [0 0 pi/2 pi/2]);          %test setting joint angles
setangles(m, [0 pi/4 pi/4 pi/2]);
setangles(m, [-pi/4 0 pi/2 pi/2]);

settorque(m, 0);                        %set torque to 0 so can position end effector
q = getangles(m);                       %read current joint angles
setangles(m, q);                        %return to saved angles

%% Part 4.2
setangles(m, [0 0 pi/2 pi/2]);          %set wrist angle to pi/2
settorque(m, 0);
q1 = getangles(m);
q2 = getangles(m);
q3 = getangles(m);                      %measure joint angles at 3 different points, save in deltajoint.m

delta = fminunc(@deltajoint, [0 0 0]);  %find delta to minimize error, save in myax12.m
q_error = deltajoint(delta)             %check error value

%% Part 4.3
mysegment                               %call mysegment script to draw line
mycircle                                %call mycircle script to draw circle
mycreative                              %call mycreative script to write the word "Hi"
    