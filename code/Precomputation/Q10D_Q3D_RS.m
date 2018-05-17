function [sD_X, sD_Z, dataX, dataZ, derivX, derivZ, TEB] = ...
  Q10D_Q3D_RS(gNX, gNZ, dt, tMax, extraArgs)
% Computes the tracking error bound and optimal control policy for a 10D
% quadrotor tracking a 3D point source.
% Inputs:
%   gNX  - Number of grid points in x subsystem (y is symmetric)
%   gNZ  - Number of grid points in z subsystem
%   dt   - Time step
%   tMax - Max time to compute
%   extraArgs - other useful inputs:
%               .accuracy = accuracy of computation (defaults to medium)
%               .targetType = type of initial implicit surface function
%                   g(x) (defaults to quadratic because we get smoother
%                   gradients)
%               .visualize = whether or not to visualize results (defaults
%                   to no)
%               .uMax = max control bounds
%               .uMin = min control bounds
%               .dMax = max disturbance bounds
%
% Outputs:
%   sD_X   - SchemeData in X
%   sD_Z   - SchemeData in Z
%   dataX  - Value function in x-subsystem
%   dataZ  - Value function in z-subsystem
%   derivX- Spatial gradients of x-subsystem value function
%   derivZ - Spatial gradients of z-subsystem value function
%   tauX   - time stamps for computing x-subsystem
%   tauZ   - time stamps for computing z-subsystem
%   TEB    - Tracking error bound

% number of grid points in each dimension
if nargin < 1
  gNX = [61 61 41 41];
  gNZ = [101 101];
end

% time step
if nargin < 2
  dt = 0.5;
end

% max # of seconds to compute back in time (should converge before this)
if nargin<3
  tMax = 10;
end

t0 = 0;
tau = t0:dt:tMax;

% set up extraArgs stuff
if nargin<4
  extraArgs = [];
end

if isfield(extraArgs,'accuracy')
  accuracy = extraArgs.accuracy;
else
  accuracy = 'veryHigh';
end

if isfield(extraArgs,'targetType')
  targetType = extraArgs.targetType;
else
  targetType = 'quadratic';
end

if isfield(extraArgs,'visualize')
  visualize = extraArgs.visualize;
else
  visualize = 1;
end


%% Set up grid
gMinX = [-2; -2; -45*pi/180; -4];
gMaxX = [ 2;  2;  45*pi/180;  4];
gMinZ = [-2; -2];
gMaxZ = [ 2;  2];

sD_X.grid = createGrid(gMinX, gMaxX, gNX);
sD_Z.grid = createGrid(gMinZ, gMaxZ, gNZ);



%% Parameters
gravity = 9.81;

% the 1st, 3rd, and 4th entries are the control of the planner
% the 2nd, 4th, and 6th entries are the control of the tracker
% see the dynamics folder for more info
if isfield(extraArgs,'uMax')
  uMax = extraArgs.uMax;
else
uMax = [.5; 20/180*pi; .5; 20/180*pi; 0.5; 1.5*gravity];
end

if isfield(extraArgs,'uMin')
  uMin = extraArgs.uMin;
else
  uMin = [-.5; -20/180*pi; -.5; -20/180*pi; -0.5; 0];
end

if isfield(extraArgs,'dMax')
  dMax = extraArgs.dMax;
else
  dMax = [0.1; 0.1; 0.1];
end
dMin = -dMax;

if isfield(extraArgs,'keepLast')
  keepLast = extraArgs.keepLast;
else
  keepLast = 1;
end

% dMax = [1/72*pi; 1/72*pi; 1/72*pi];
% dMin = -dMax;

uMode = 'min'; %10D trying to min
dMode = 'max'; %3D trying to max

sD_X.accuracy = accuracy;
sD_Z.accuracy = accuracy;
sD_X.uMode = uMode;
sD_X.dMode = dMode;
sD_Z.uMode = uMode;
sD_Z.dMode = dMode;
%% initial data

%generally we actually use a quadratic cost beacuse it makes the gradients
%prettier. We then take the square root at the end.
if strcmp(targetType,'oneNorm')
  dataX0 = shapeRectangleByCorners(...
      sD_X.grid,[0 -Inf -Inf -Inf],[0 Inf Inf Inf]);
  dataZ0 = shapeRectangleByCorners(sD_Z.grid,[0 -Inf],[0 Inf]);
  
elseif strcmp(targetType,'quadratic')
  dataX0 = sD_X.grid.xs{1}.^2;
  dataZ0 = sD_Z.grid.xs{1}.^2;
else
  error('what targetType?')
end

if visualize
  figure(1)
  clf
  subplot(2,1,1)
  hZ0 = surf(sD_Z.grid.xs{1},sD_Z.grid.xs{2},sqrt(dataZ0));
  xlabel('$z_r$','Interpreter','latex','FontSize',20)
  ylabel('$v_z$','Interpreter','latex','FontSize',20)
  
  subplot(2,1,2)
  [g2DX,data2DX]=proj(sD_X.grid,sqrt(dataX0),[0 0 1 1],[0 0]);
  hX0 = surf(g2DX.xs{1},g2DX.xs{2},data2DX);
  xlabel('$x_r$','Interpreter','latex','FontSize',20)
  ylabel('$v_x$','Interpreter','latex','FontSize',20)
end
%% Dynamical systems and subsystems
Xdims = 1:4;
Zdims = 9:10;

sD_X.dynSys = Q10D_Q3D_Rel(zeros(10,1), uMin, uMax, dMin, dMax, Xdims);
sD_Z.dynSys = Q10D_Q3D_Rel(zeros(10,1), uMin, uMax, dMin, dMax, Zdims);


%% Z subsystem Solver parameters

if isfield(extraArgs,'stopConverge')
  HJIextraArgs.stopConverge = extraArgs.stopConverge;
  if isfield(extraArgs,'convergeThreshold')
    HJIextraArgs.convergeThreshold = extraArgs.convergeThreshold;
  end
else
  HJIextraArgs.stopConverge = 1;
  HJIextraArgs.convergeThreshold = .1;
end

if visualize
  HJIextraArgs.visualize = 1;
  %HJIextraArgs.RS_level = 1;
  HJIextraArgs.visualizeValueFunction = 1;
  HJIextraArgs.fig_num = 2;
  HJIextraArgs.deleteLastPlot = 1;
  figure(HJIextraArgs.fig_num)
  clf
end
HJIextraArgs.keepLast = keepLast;

%% Run z subsystem
[dataZ, tauZ] = HJIPDE_solve(dataZ0, tau, sD_Z, 'maxVOverTime', HJIextraArgs);


%% X subsystems solver parameters
if visualize
  HJIextraArgs.plotData.plotDims = [1 1 0 0];
  HJIextraArgs.plotData.projpt = [0 0];
  HJIextraArgs.RS_level = 1;
  HJIextraArgs.fig_num = 3;
  figure(HJIextraArgs.fig_num)
  clf
end

%% Run x subsystem
[dataX, tauX] = HJIPDE_solve(dataX0, tau, sD_X, 'maxVOverTime', HJIextraArgs);

%% Find tracking error bound
dataX_last = dataZ(:,:,:,:,end);
dataZ_last = dataZ(:,:,end);
TEB_Z = min(dataZ_last);
TEB_X = min(dataX_last);
TEB = max(TEB_Z,TEB_X);

if strcmp(targetType,'quadratic')
  TEB = sqrt(TEB);
end

%% Visualize
if visualize
  figure(4)
  clf
  subplot(2,1,1)
  hZ = surf(sD_Z.grid.xs{1}, sD_Z.grid.xs{2}, sqrt(dataZ_last));
  xlabel('$z_r$','Interpreter','latex','FontSize',20)
  ylabel('$v_z$','Interpreter','latex','FontSize',20)
  
  subplot(2,1,2)
  [g2DX,data2DX]=proj(sD_X.grid, sqrt(dataX_last), [0 0 1 1],[0 0]);
  hX = surf(g2DX.xs{1},g2DX.xs{2},data2DX);
  xlabel('$x_r$','Interpreter','latex','FontSize',20)
  ylabel('$v_x$','Interpreter','latex','FontSize',20)
end
%% compute gradients (for controller)
derivX = computeGradients(sD_X.grid, dataX_last);
derivZ = computeGradients(sD_Z.grid, dataZ_last);

%% save
save(['Quad10D_g' num2str(gNZ(1)) '_dt0' num2str(dt*100) '_t' ...
    num2str(tMax) '_' accuracy '_' targetType '.mat'], 'TEB','sD_X', ...
    'sD_Z', 'dataX', 'dataZ', 'derivX','derivZ', '-v7.3')
%save(sprintf('%s_%f.mat', mfilename, now), 'sD_X', 'sD_Z', 'dataX', 'dataZ', ...
%  'tau', '-v7.3')
end