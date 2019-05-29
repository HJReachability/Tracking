%function [data_big, data_small, data_switch]=example_3d(gN)
%Q10D_Q4D_RS Summary of this function goes here
%   Detailed explanation goes here
accuracy = 'low';
small = .2;
%if nargin < 1
  gN = [111; 111; 51];
%end

%% compute TEB1
% Grid and cost
gMin = [-5; -5; -pi];
gMax = [ 5;  5;  pi];
sD_big.grid = createGrid(gMin, gMax, gN,3);

data0 = sD_big.grid.xs{1}.^2 + sD_big.grid.xs{2}.^2;

% Dynamical system
uMin = [-8];
uMax = [8];

pMin = [-1; -1];
pMax = [1; 1];

dMax = [0; 0];
dMin = [0; 0];

vel = 3;

dims = 1:3;

sD_big.dynSys = P3D_Q2D_Rel([], uMin, uMax, pMin, pMax, dMin, dMax, vel, dims);

% Otherparameters
sD_big.uMode = 'min';
sD_big.dMode = 'max';
sD_big.accuracy = accuracy;

extraArgs.visualize.valueFunction = 1;
extraArgs.visualize.sliceLevel = 3;
extraArgs.visualize.figNum = 2;
extraArgs.visualize.plotData.plotDims = [1 1 0];
extraArgs.visualize.plotData.projpt = 0;
extraArgs.visualize.deleteLastPlot = true;
  dt = 0.02;
  tMax = 5;
  tau = 0:dt:tMax;
  
  extraArgs.stopConverge = true;
  extraArgs.convergeThreshold = dt;%0.5*dt;
  extraArgs.keepLast = 1;
  [data_big, tau] = HJIPDE_solve(data0, tau, sD_big, 'maxVWithV0', extraArgs);
 TEB_big = min(sqrt(data_big(:)))+small;
  
  %% compute TEB2
  gMin = [-5; -5; -pi];
gMax = [ 5;  5;  pi];
sD_small.grid = createGrid(gMin, gMax, gN,3);

data0 = sD_small.grid.xs{1}.^2 + sD_small.grid.xs{2}.^2;

% Dynamical system
uMin = [-8];
uMax = [8];

pMin = [-.1; -.1];
pMax = [.1; .1];

dMax = [0; 0];
dMin = [0; 0];

vel = 3;

dims = 1:3;

sD_small.dynSys = P3D_Q2D_Rel([], uMin, uMax, pMin, pMax, dMin, dMax, vel, dims);

% Otherparameters
sD_small.uMode = 'min';
sD_small.dMode = 'max';
sD_small.accuracy = accuracy;

%extraArgs.visualize. = true;
extraArgs.visualize.sliceLevel = 3;
extraArgs.visualize.figNum = 2;
extraArgs.visualize.plotData.plotDims = [1 1 0];
extraArgs.visualize.plotData.projpt = 0;
extraArgs.visualize.deleteLastPlot = true;
  dt = 0.02;
  tMax = 5;
  tau = 0:dt:tMax;
  
  extraArgs.stopConverge = true;
  extraArgs.convergeThreshold = dt;%0.5*dt;
  extraArgs.keepLast = 1;
  [data_small, tau] = HJIPDE_solve(data0, tau, sD_small, 'max_data0', extraArgs);
 
  TEB_small = min(sqrt(data_small(:)))+small;
  %% compute switching
    dt = 0.02;
  tMax = 20;
  tau = 0:dt:tMax;
  
sD_switch.dynSys = sD_small.dynSys; % set dynamic system to smaller bound (i.e. smaller planner dynamics/controls)
sD_switch.uMode = 'min'; % tracker tries to minimize
sD_switch.dMode = 'max'; % planner tries to maximize
sD_switch.accuracy = accuracy;

sD_switch.grid = sD_big.grid;
data0 = sqrt(data_small)-TEB_small;
dataF = sqrt(data_big) - TEB_big;
figure(4)
clf
cylinder_small = visSetIm(sD_small.grid, data0,'b',0);
hold on
cylinder_big = visSetIm(sD_big.grid, dataF, 'r',0);
cylinder_big.FaceAlpha = .3;

figure(5)
clf
[g_p,data_big_p] = proj(sD_big.grid, dataF, [0 0 1],'min');
box_big = visSetIm(g_p,data_big_p,'r',0);
%box_big.FaceAlpha = .3;
hold on

[g_p,data_small_p] = proj(sD_small.grid, data0, [0 0 1],'min');
box_small = visSetIm(g_p,data_small_p,'b',0);


extraArgs_switch.visualize = true;
extraArgs_switch.fig_num = 3;
extraArgs_switch.stopSetInclude = dataF;
    [data_switch, tau] = HJIPDE_solve(data0, tau, ...
        sD_switch, 'min_data0', extraArgs_switch);
    
%% visualize
figure(4)
hold on
cylinder_switch = visSetIm(sD_switch.grid, data_switch, 'k',0);
cylinder_switch.FaceAlpha = .2;

figure(5)
%project everything into position
% [g_p,data_big_p] = proj(sD_big.grid, dataF, [0 0 1],'min');
% box_big = visSetIm(g_p,data_big_p,'r',0);
% %box_big.FaceAlpha = .3;
% hold on
% 
% [g_p,data_small_p] = proj(sD_small.grid, data0, [0 0 1],'min');
% box_small = visSetIm(g_p,data_small_p,'b',0);
hold on
[g_p,data_switch_p] = proj(sD_switch.grid, data_switch, [0 0 1],'min');
box_switch = visSetIm(g_p,data_switch_p,'k',0);
%box_switch.FaceAlpha = .2;


%% compute trajectory to enter smaller set
% sD_switch.dynSys.x = [0, -1.75,0];
% extraArgTraj.visualize = 1;
% extraArgTraj.fig_num = 5;
% extraArgTraj.projDim = [1,2];
% extraArgTraj.uMode = 'min';
% [traj, traj_tau] = computeOptTraj(sD_switch.grid, data_switch, tau, sD_switch.dynSys, extraArgTraj)
%end

