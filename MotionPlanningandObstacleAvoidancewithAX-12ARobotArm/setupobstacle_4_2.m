% Units are centimetres
% Obstacle 1: ground plane
obs{1}.rho0 = 50;
obs{1}.type = 'ground';
% Obstacle 2: Cylinder 1
obs{2}.R = 30;
obs{2}.c = [165; -165];
obs{2}.rho0 = 50;
obs{2}.h = 160;
obs{2}.type = 'cyl';
% Obstacle 3: Cylinder 2
obs{3}.R = 30;
obs{3}.c = [0;130];
obs{3}.rho0 = 50;
obs{3}.h = 160;
obs{3}.type = 'cyl';
