function [gX, gZ, dataX, dataZ, dataX0, dataZ0, tauX, tauZ]=Quad10D_Rel_RS(gN, dt, tMax, accuracy, targetType, visualize, gX,gZ)
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
  tMax = 50;
end

t0 = 0;
tau = t0:dt:tMax;

if nargin <4
  accuracy = 'low';
end

if nargin<5
  targetType = 'oneNorm';
end

if nargin<6
  visualize = 0;
end

if nargin<7
  % Grid
gMinX = [-5; -5; -35*pi/180; -1];
gMaxX = [ 5;  5;  35*pi/180;  1];
gMinZ = [-5; -5];
gMaxZ = [ 5;  5];

gX = createGrid(gMinX, gMaxX, gN*ones(4,1));
gZ = createGrid(gMinZ, gMaxZ, gN*ones(2,1));
end


%% Common parameters


gravity = 9.81;
uMax = [.5; 20/180*pi; .5; 20/180*pi; 0.5; 1.5*gravity];
uMin = [-.5; -20/180*pi; -.5; -20/180*pi; -0.5; 0];
dMax = [.1; .1; 0.1];
dMin = -dMax;

% dMax = [1/72*pi; 1/72*pi; 1/72*pi];
% dMin = -dMax;

uMode = 'max'; %10D trying to max
dMode = 'min'; % disturbance trying to min


%% initial data

if strcmp(targetType,'oneNorm')
  dataX0 = -shapeRectangleByCorners(gX,[0 -Inf -Inf -Inf],[0 Inf Inf Inf]);
  dataZ0 = -shapeRectangleByCorners(gZ,[0 -Inf],[0 Inf]);
  
elseif strcmp(targetType,'quadratic')
  dataX0 = -gX.xs{1}.^2;
  dataZ0 = -gZ.xs{1}.^2;
else
  error('what targetType?')
end

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
sD_X.uMode = uMode;
sD_X.dMode = dMode;
sD_Z.uMode = uMode;
sD_Z.dMode = dMode;

%% Additional solver parameters
extraArgs.targets = dataZ0;
extraArgs.stopConverge = 1;
extraArgs.convergeThreshold = 0.01;
extraArgs.keepLast = 1;

if visualize
extraArgs.visualize = 1;
extraArgs.RS_level = -5;
extraArgs.fig_num = 4;
figure(extraArgs.fig_num)
clf
end

[dataZ, tauZ] = HJIPDE_solve(dataZ0, tau, sD_Z, 'none', extraArgs);



if visualize
extraArgs.plotData.plotDims = [1 1 1 0];
extraArgs.plotData.projpt = [0];
extraArgs.deleteLastPlot = 1;
extraArgs.fig_num = 5;
figure(extraArgs.fig_num)
clf
end

extraArgs.targets = dataX0;
extraArgs.stopConverge = 1;
extraArgs.convergeThreshold = 0.01;
extraArgs.keepLast = 1;

[dataX, tauX] = HJIPDE_solve(dataX0, tau, sD_X, 'none', extraArgs);




%save(sprintf('%s_%f.mat', mfilename, now), 'gX', 'gZ', 'dataX', 'dataZ', ...
%  'tau', '-v7.3')
end