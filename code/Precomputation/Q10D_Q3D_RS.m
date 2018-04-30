function [sD_X, sD_Z, dataX, dataZ, dataX0, dataZ0, derivX,derivZ,tauX, tauZ, TEB, runtime_X, runtime_Z] = ...
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
%   dataX0 - Initial implicit surface function in x-subsystem
%   dataZ0 - Initial implicit surface function in z-subsystem
%   tauX   - time stamps for computing x-subsystem
%   tauZ   - time stamps for computing z-subsystem
%   TEB    - Tracking error bound

% number of grid points in each dimension
if nargin < 1
  gNX = [101 101 ceil(101/8) ceil(101/5)];
  gNZ = [101 101];
end

% time step
if nargin < 2
  dt = 0.01;
end

% max # of seconds to compute back in time (should converge before this)
if nargin<3
  tMax = 5;
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
  accuracy = 'medium';
end

if isfield(extraArgs,'targetType')
  targetType = extraArgs.targetType;
else
  targetType = 'quadratic';
end

if isfield(extraArgs,'visualize')
  visualize = extraArgs.visualize;
else
  visualize = 0;
end


%% Set up grid
gMinX = [-5; -5; -35*pi/180; -1];
gMaxX = [ 5;  5;  35*pi/180;  1];
gMinZ = [-5; -5];
gMaxZ = [ 5;  5];

sD_X.grid = createGrid(gMinX, gMaxX, gNX);
sD_Z.grid = createGrid(gMinZ, gMaxZ, gNZ);



%% Parameters
gravity = 9.81;
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
  dMax = [0; 0; 0];
end
dMin = -dMax;

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

if strcmp(targetType,'oneNorm')
  dataX0 = shapeRectangleByCorners(sD_X.grid,[0 -Inf -Inf -Inf],[0 Inf Inf Inf]);
  dataZ0 = shapeRectangleByCorners(sD_Z.grid,[0 -Inf],[0 Inf]);
  
elseif strcmp(targetType,'quadratic')
  dataX0 = sD_X.grid.xs{1}.^2;
  dataZ0 = sD_Z.grid.xs{1}.^2;
else
  error('what targetType?')
end

if visualize
  fZ = 1;
  figure(fZ)
  clf
  hZ = surf(sD_Z.grid.xs{1},sD_Z.grid.xs{2},dataZ0);
  xlabel('$z_r$','Interpreter','latex','FontSize',20)
  ylabel('$v_z$','Interpreter','latex','FontSize',20)
  
  fX = 3;
  figure(fX)
  clf
  [g2DX,data2DX]=proj(sD_X.grid,dataX0,[0 0 1 1],'max');
  hX = surf(g2DX.xs{1},g2DX.xs{2},data2DX);
  xlabel('$x_r$','Interpreter','latex','FontSize',20)
  ylabel('$v_x$','Interpreter','latex','FontSize',20)
end
%% Dynamical systems and subsystems
Xdims = 1:4;
Zdims = 9:10;

sD_X.dynSys = Q10D_Q3D_Rel(zeros(10,1), uMin, uMax, dMin, dMax, Xdims);
sD_Z.dynSys = Q10D_Q3D_Rel(zeros(10,1), uMin, uMax, dMin, dMax, Zdims);


%% Additional solver parameters


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
  HJIextraArgs.RS_level = 2;
  HJIextraArgs.fig_num = fZ+1;
  figure(HJIextraArgs.fig_num)
  clf
end

tic
[dataZ, tauZ] = HJIPDE_solve(dataZ0, tau, sD_Z, 'maxVOverTime', HJIextraArgs);
runtime_Z = toc;

if visualize
  HJIextraArgs.plotData.plotDims = [1 1 0 0];
  HJIextraArgs.plotData.projpt = [0 0];
  HJIextraArgs.RS_level = 4;
  HJIextraArgs.fig_num = fX+1;
  figure(HJIextraArgs.fig_num)
  clf
end

tic
[dataX, tauX] = HJIPDE_solve(dataX0, tau, sD_X, 'maxVOverTime', HJIextraArgs);
runtime_X = toc;

TEB_Z = min(dataZ(:));
TEB_X = min(dataX(:));
TEB = max(TEB_Z,TEB_X);

if strcmp(targetType,'quadratic')
  TEB = sqrt(TEB);
end

derivX = computeGradients(sD_X.grid,dataX);
derivZ = computeGradients(sD_Z.grid,dataZ);

save(['Quad10D_g' num2str(gNZ(1)) '_dt0' num2str(dt*100) '_t' ...
    num2str(tMax) '_' accuracy '_' targetType '.mat'], 'sD_X', ...
    'sD_Z', 'dataX', 'dataZ','derivX','derivZ', 'tau', '-v7.3')
%save(sprintf('%s_%f.mat', mfilename, now), 'sD_X', 'sD_Z', 'dataX', 'dataZ', ...
%  'tau', '-v7.3')
end