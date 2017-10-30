%% part 4.1 The Attractive Field

DH = [
    1.5708         0         0   76.0000;
         0   43.2300         0  -23.6500;
    1.5708         0         0         0;
   -1.5708         0         0   43.1800;
    1.5708         0         0         0;
         0         0         0   20.0000];          %define the DH table

myrobot = mypuma560(DH);                            %create the robot structure

H1 = eul2tr([0 pi pi/2]);
H1(1:3,4)=100*[-1; 3; 3;]/4;
q1 = inverse(H1,myrobot);                           
H2 = eul2tr([0 pi -pi/2]);
H2(1:3,4)=100*[3; -1; 2;]/4;
q2 = inverse(H2,myrobot);
tau = att(q1,q2,myrobot)                            %calculate joint torques resulting from attractive potential


%% part 4.2 Motion Planning without Obstacles

qref = motionplan(q1,q2,0,10,myrobot,[],0.01);      %generate motion plan based off attractive potential (no obstacles)
t = linspace(0,10,300);
q = ppval(qref,t)';
plot(myrobot,q)


%% part 4.3 Motion Planning with Obstacles

setupobstacle
q3 = 0.9*q1 + 0.1*q2;
tau = rep(q3, myrobot, obs{1})                      %calculate joint torques resulting from repulsive potential for a cylindrical obstacle

q = [pi/2 pi 1.2*pi 0 0 0];                         %calculate joint torques resulting from repulsive potential for a spherical obstacle
tau = rep(q, myrobot, obs{6})

hold on
axis([-1 1 -1 1 0 2]*100)
view(-32, 50)
plotobstacle(obs);
qref = motionplan(q1, q2, 0, 10, myrobot, obs, 0.01);      %generate motion plan based off attractive and repulsive potentials     
%NOTE: This function takes about 4 minutes to run, but does eventually finish
t = linspace(0, 10, 300);
q = ppval(qref,t)';
plot(myrobot,q)
hold off