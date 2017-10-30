%% part 4.1 Calibration of DH parameters

DH = [
            pi/2 -55 0 142;
            0 172 0 0;
            pi/2 23 0 0;
            0 5 0 285];          %define the DH table

delta = [ 18.5770  109.4362   93.2059  158.8121];        %delta used in lab

myrobot = myax12(DH);                            %create the robot structure
%or equivalently, myrobot = myax12_search(delta);

%% part 4.2 Initial Motion Planning In Simulation

p1 = [0 -200 5]; q1 = inverse_ax12(myrobot, p1);
p2 = [200 -20 20]; q2 = inverse_ax12(myrobot, p2);
p3 = [-220 220 40]; q3 = inverse_ax12(myrobot, p3);

setupobstacle_4_2;

qref = motionplan(q1',q2',0,10,myrobot,obs,0.0962);
t = linspace(0,10,300);
sim1 = ppval(qref,t)';
qref = motionplan(q2',q3',0,10,myrobot,obs,0.0225);
sim2 = ppval(qref,t)';
q = [sim1; sim2];
hold on
plotobstacle(obs);
plot(myrobot,q);


%% part 4.3 Initial Motion Planning with AX12-A
setangles(m, q1);           %put robot in initial position
setgripper(m, 0);           %open gripper, then place marshmellow
setgripper(m, 75);          %grip marshmellow
pause(0.5);
for i = 1:numel(q(:,1))     %execute motion plan
   setangles(m,q(i,:));    
   pause(0.02) 
end
setgripper(m, 0)            %drop marshmellow

%% part 4.4 Creative Motion Planning with AX12-A

setupobstacle               %our creative obstacles
q_cup = [0.2612;0.8805;-0.1178;1.5708 ];
q_drop = [2.214045603848275;0.848802702630055;0.956181357782050;3.141592653589793];
q_drop1 = [1.518640980006785;0.731197508892156;-0.086925577980186;3.124207537993756];
q_mid = [-0.685178085255586;0.966407896367954;0.158511348081516;1.570796326794897];
q_pick = [0.260776733940559;0.792556740407581;-0.373268658385506;3.129320807286708];

qref = motionplan(q1',q_mid',0,10,myrobot,obs,0.01);
path0 = ppval(qref,t)';
qref = motionplan(q_mid',q_cup',0,10,myrobot,obs,0.01);
path1 = ppval(qref,t)';
qref = motionplan(q_pick',q_drop1',0,10,myrobot,obs,0.01);
path2 = ppval(qref,t)';
qref = motionplan(q_drop1',q_drop',0,10,myrobot,obs,0.01);
path3 = ppval(qref,t)';

motionplan_creative