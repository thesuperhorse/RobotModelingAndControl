% Script to have the manipulator draw a line

DH = [
        pi/2 -55 0 142;
        0 172 0 0;
        pi/2 23 0 0;
        0 5 0 285];
    
myrobot = myax12(DH);                       %create robot

x = ones(1,100)*200;                        %x and y coordinates of line
y = linspace(-100,100,100);             
z = zeros(1,100)+45;                        %z offset to keep pen on page

X = [x; y; z];

setangles(m, [0 0 pi/2 pi/2]);              %set intial position
pause(2);

for i=1:numel(X(1,:))
    q = inverse_ax12(myrobot,X(:,i));       %execute trajectory
    setangles(m,q);
    pause(0.1);
end