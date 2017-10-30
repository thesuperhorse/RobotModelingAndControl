% Script to have the manipulator draw a circle

DH = [
        pi/2 -55 0 142;
        0 172 0 0;
        pi/2 23 0 0;
        0 5 0 285];
    
myrobot = myax12(DH);               %create  robot

theta = linspace(0, 2*pi, 100);     


x = 50*cos(theta)+190;              %x and y coordinates of the circle
y = 50*sin(theta)+160;
z = zeros(1,100)+45;                %offset of z to set pen on page

X = [x; y; z];


setangles(m, [0 0 pi/2 pi/2]);      %set initial position
pause(2);
for i=1:numel(X(1,:))
    
    q = inverse_ax12(myrobot,X(:,i));       %executae trajectory
    setangles(m,q);
    pause(0.1);
end