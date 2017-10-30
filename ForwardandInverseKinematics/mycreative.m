% Script to have the manipulator write the word "Hi"


DH = [
        pi/2 -55 0 142;
        0 172 0 0;
        pi/2 23 0 0;
        0 5 0 285];
    
myrobot = myax12(DH);               %create robot

x1 = linspace(225, 270, 100);       %draws one side of H
y1 = zeros(1,100);
z1 = zeros(1,100)+65;

X = [x1; y1; z1];

x1 = ones(1,100)*255;               %draws cross of H
y1 = linspace(0,25,100);
z1 = zeros(1,100)+80;

X = [X, [x1; y1; z1]];

x1 = linspace(225, 270, 100);       %draws other side of H
y1 = zeros(1,100)+25;
z1 = zeros(1,100)+65;

X = [X, [x1; y1; z1]];

x1 = linspace(270, 255, 20);        %raise pen to move to next letter
y1 = linspace(25, 65, 20);
z1 = zeros(1,20)+150;

X = [X, [x1; y1; z1]]; 

x1 = ones(1,20)*255;                %lower pen to draw bottom of i
y1 = ones(1,20)*65;
z1 = linspace(150,65,20);

X = [X, [x1; y1; z1]];

x1 = linspace(255, 270, 50);        %draw bottom of i
y1 = zeros(1,50)+65;
z1 = zeros(1,50)+65;

X = [X, [x1; y1; z1], [270;65;95], [240;65;95]];

setangles(m, [0 0 pi/2 pi/2]);      %set initial position
pause(2);

for i=1:numel(X(1,:))

    q = inverse_ax12(myrobot,X(:,i));   %execute trajectory
    setangles(m,q);
    pause(0.1);
end

pause(1)

x1 = ones(1,20)*240;                    
y1 = ones(1,20)*65;
z1 = linspace(95, 65, 20);

X = [[x1; y1; z1], [x1;y1;z1(end:-1:1)]];   %lower and raise pen to draw dot of i  


for i=1:numel(X(1,:))

    q = inverse_ax12(myrobot,X(:,i));   %execute dot drawing trajectory
    setangles(m,q);
    pause(0.1);
end

setangles(m, [pi/2; q(2:end)]);         %expose the masterpiece (move pen away from the word)
