% Units are centimetres
% Obstacle 1: ground plane
obs{1}.rho0 = 50;
obs{1}.type = 'ground';
% Obstacle 2: Cylinder 1
obs{2}.R = 30;
obs{2}.c = [165; -165];
obs{2}.rho0 = 50;
obs{2}.h = 120;
obs{2}.type = 'cyl';
% Obstacle 3: Cylinder 2
obs{3}.R = 30;
obs{3}.c = [10.5;-8]*13;
obs{3}.rho0 = 50;
obs{3}.h = 120;
obs{3}.type = 'cyl';
% Obstacle 4: Cylinder 3
obs{4}.R = 30;
obs{4}.c = [17;-7.8]*13;
obs{4}.rho0 = 50;
obs{4}.h = 120;
obs{4}.type = 'cyl';
% Obstacle 5
obs{5}.R = 100;
obs{5}.c = [-220; 220];
obs{5}.rho0 = 50;
obs{5}.h = 240;
obs{5}.type = 'cyl';
% Obstacle 6
obs{5}.R = 10;
obs{5}.c = [8; -7]*13;
obs{5}.rho0 = 50;
obs{5}.h = 180;
obs{5}.type = 'cyl';
