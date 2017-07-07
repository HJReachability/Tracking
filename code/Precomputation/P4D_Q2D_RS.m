function [data,tau,sD,teb]=P4D_Q2D_RS(gN, visualize)

if nargin < 1
  gN = [55; 55; 25];
end

if nargin < 2
  visualize = true;
end

%% Grid and cost

% Grid Bounds
gMin = [-1.5; -1.5; -pi];
gMax = [ 1.5;  1.5;  pi];

% createGrid takes in grid bounds, grid number, and periodic dimensions
sD.grid = createGrid(gMin, gMax, gN,3);

% Cost Function
data0 = sD.grid.xs{1}.^2 + sD.grid.xs{2}.^2;

% obstacles
extraArgs.obstacles = -data0;

if visualize
  % to visualize initial function over 2D grid, project onto 2D
  [g2D, data02D] = proj(sD.grid,data0,[0 0 1],'max');
  
  figure(1)
  clf
  subplot(1,2,1)
  
  % plot data02D over grid in x and y
  surf(g2D.xs{1}, g2D.xs{2}, sqrt(data02D))
end

%% Dynamical system

% min and max turn rate for tracker
uMin = [-8];
uMax = [8];

% min and max velocities for planner (in x and y)
pMin = [-.5; -.5];
pMax = [.5; .5];

% min and max disturbance for wind (in x and y)
dMax = [0; 0];
dMin = [0; 0];

% fixed velocity of tracker
vel = 3;

% number of dimensions in the system
dims = 1:3;

% create dynamic system
sD.dynSys = P3D_Q2D_Rel([], uMin, uMax, pMin, pMax, dMin, dMax, vel, dims);

%% Other Parameters

% is tracker minimizing or maximizing?
sD.uMode = 'min';

% is planner minimizing or maximizing?
sD.dMode = 'max';

% how high accuracy?
sD.accuracy = 'medium';

if visualize
  % set visualize to true
  extraArgs.visualize = true;
  
  % set slice of function to visualize
  extraArgs.RS_level = 5;
  
  % figure number
  extraArgs.fig_num = 2;
  %extraArgs.plotData.plotDims = [0 0 1];
  %extraArgs.plotData.projpt = 0;
  % delete previous time step's plot
  extraArgs.deleteLastPlot = true;
end

% time step
dt = 0.1;

% Max time
tMax = .75;

% Vector of times
tau = 0:dt:tMax;

% stop when function has converged
extraArgs.stopConverge = true;

% converges when function doesn't change by more than dt each time step
extraArgs.convergeThreshold = dt;

% solve backwards reachable set
[data, tau] = HJIPDE_solve(data0, tau, sD, 'none', extraArgs);

% largest cost along all time (along the entire 4th dimension which is
% time)
data = max(data,[],4);

if visualize
  figure(1)
  subplot(1,2,2)
  [g2D, data2D] = proj(sD.grid, data, [0 0 1], 'min');
  data2D = sqrt(data2D);
  surf(g2D.xs{1}, g2D.xs{2}, data2D)
  
  figure(2)
  clf
  alpha = .2;
  small = .05;
  
  levels = [.5, .75, 1];
  
  [g2D, data2D] = proj(sD.grid,data,[0 0 1],0);%'max');
  
  subplot(2,3,1)
  h0 = visSetIm(sD.grid, sqrt(data0), 'blue', levels(1)+small);
  h0.FaceAlpha = alpha;
  hold on
  h = visSetIm(sD.grid, sqrt(data), 'red', levels(1));
  axis([-levels(3)-small levels(3)+small ...
    -levels(3)-small levels(3)+small -pi pi])
  axis square
  
  subplot(2,3,4)
  h0 = visSetIm(g2D, sqrt(data02D), 'blue', levels(1)+small);
  hold on
  h = visSetIm(g2D, sqrt(data2D), 'red', levels(1));
  axis([-levels(3)-small levels(3)+small ...
    -levels(3)-small levels(3)+small])
  title(['R = ' num2str(levels(1))])
  axis square
  
  subplot(2,3,2)
  h0 = visSetIm(sD.grid, sqrt(data0), 'blue', levels(2)+small);
  h0.FaceAlpha = alpha;
  hold on
  h = visSetIm(sD.grid, sqrt(data), 'red', levels(2));
  axis([-levels(3)-small levels(3)+small ...
    -levels(3)-small levels(3)+small -pi pi])
  title(['t = ' num2str(tau(end)) ' s'])
  axis square
  
  subplot(2,3,5)
  h0 = visSetIm(g2D, sqrt(data02D), 'blue', levels(2)+small);
  hold on
  h = visSetIm(g2D, sqrt(data2D), 'red', levels(2));
  axis([-levels(3)-small levels(3)+small ...
    -levels(3)-small levels(3)+small])
  title(['R = ' num2str(levels(2))])
  axis square
  
  subplot(2,3,3)
  h0 = visSetIm(sD.grid, sqrt(data0), 'blue', levels(3)+small);
  h0.FaceAlpha = alpha;
  hold on
  h = visSetIm(sD.grid, sqrt(data), 'red', levels(3));
  axis([-levels(3)-small levels(3)+small ...
    -levels(3)-small levels(3)+small -pi pi])
  axis square
  
  subplot(2,3,6)
  h0 = visSetIm(g2D, sqrt(data02D), 'blue', levels(3)+small);
  hold on
  h = visSetIm(g2D, sqrt(data2D), 'red', levels(3));
  axis([-levels(3)-small levels(3)+small ...
    -levels(3)-small levels(3)+small])
  title(['R = ' num2str(levels(3))])
  axis square
  
  set(gcf,'Color','white')
end

%tracking error bound
teb = sqrt(min(data(:)))
end

