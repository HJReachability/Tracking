function [gX, gZ, dataX, dataZ,tau]=Quad10D_Rel_RS(gN, dt, accuracy,gX,gZ)
% DubinsCar_RS()
%     Compares reachable set/tube computation using direct and decomposition
%     methods

if nargin < 1
  gN = 20;
end

if nargin < 2
  dt = 0.01;
end

if nargin<3
  accuracy = 'low';
end

if nargin<4
  % Grid
gMinX = [-20; -20; -100*pi/180; -20];
gMaxX = [ 20;  20;  100*pi/180;  20];
gMinZ = [-20; -20];
gMaxZ = [ 20;  20];
gX = createGrid(gMinX, gMaxX, gN*ones(4,1));
gZ = createGrid(gMinZ, gMaxZ, gN*ones(2,1));
end

gravity = 9.81;
uMax = [10/180*pi; 10/180*pi; 2*gravity];
uMin = [-10/180*pi; -10/180*pi; 0];
dMax = [.5; .5; .5];
dMin = -dMax;

%% Common parameters



uMode = 'max'; %10D trying to max
dMode = 'min'; % 2D trying to min
%targetLower= [gMinX; gMinX; gMinZ];
%targetUpper= [gMaxX; gMaxX; gMaxZ];
% targetLower = -inf(10, 1);
% targetUpper = inf(10, 1);
%decompose 2m radius circle into inscribed square
r = 2;

%targetLower([1 5 9]) = [-1.4142; -1.4142; -1.4142]; 
%targetUpper([1 5 9]) = [1.4142; 1.4142; 1.4142];



% Time
tMax = 20;
tau = 0:dt:tMax;

% plotting
folder = sprintf('%s_%f', mfilename, now);
system(sprintf('mkdir %s', folder));

%% Dynamical systems and subsystems
Xdims = 1:4;
Zdims = 9:10;

sD_X.dynSys = Quad10D_Rel(zeros(10,1), uMin, uMax, dMax, dMin, Xdims);
sD_Z.dynSys = Quad10D_Rel(zeros(10,1), uMin, uMax, dMax, dMin, Zdims);

%% Grids and initial conditions

sD_X.grid = gX;
sD_Z.grid = gZ;
sD_X.accuracy = accuracy;
sD_Z.accuracy = accuracy;

% dataGridX0 = shapeRectangleByCorners(sD_X.grid, sD_X.grid.min, sD_X.grid.max);
% dataBoxX0 = shapeRectangleByCorners(sD_X.grid, targetLower(Xdims), ...
%   targetUpper(Xdims));
% dataX0 = shapeRectangleByCorners(sD_X.grid, targetLower(Xdims), ...
%   targetUpper(Xdims));
% dataX0 = shapeComplement(dataX0);
% dataZ0 = shapeRectangleByCorners(sD_Z.grid, targetLower(Zdims), ...
%   targetUpper(Zdims));
% dataZ0 = shapeComplement(dataZ0);

% dataX0
 ignoreDims = [2 3 4];
 center = [0 0 0 0];
 dataX0 = zeros(gX.shape);
 for i = 1 : gX.dim
   if(all(i ~= ignoreDims))
     dataX0 = dataX0 + (gX.xs{i} - center(i)).^2;
   end
 end
 dataX0 = -sqrt(dataX0);

% dataZ0
 ignoreDims = 2;
 center = [0 0 0 0];
 dataZ0 = zeros(gZ.shape);
 for i = 1 : gZ.dim
   if(all(i ~= ignoreDims))
     dataZ0 = dataZ0 + (gZ.xs{i} - center(i)).^2;
   end
 end
 dataZ0 = -sqrt(dataZ0);


%% Additional solver parameters
sD_X.uMode = uMode;
sD_Z.uMode = uMode;
sD_X.dMode = dMode;
sD_Z.dMode = dMode;
%extraArgs.visualize = true;
%extraArgs.deleteLastPlot = true;
extraArgs.target = dataZ0;
extraArgs.stopConverge = 1;

dataZ = HJIPDE_solve(dataZ0, tau, sD_Z, 'none', extraArgs);

% extraArgs.plotData.projpt = 0;
% extraArgs.plotData.plotDims = [1 1 1 0];
extraArgs.target = dataX0;
%extraArgs.stopInit = zeros(10,1);
extraArgs.stopConverge = 1;
%extraArgs.fig_filename = sprintf('%s/', folder);

dataX = HJIPDE_solve(dataX0, tau, sD_X, 'none', extraArgs);

save(sprintf('%s_%f.mat', mfilename, now), 'gX', 'gZ', 'dataX', 'dataZ', ...
  'tau', '-v7.3')
end