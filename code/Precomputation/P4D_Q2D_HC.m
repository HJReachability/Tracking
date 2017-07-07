function [data,tau,sD,teb]=P4D_Q2D_HC(gN, visualize)

if nargin < 1
  gN = [55; 55; 25; 25];
end

if nargin < 2
  visualize = true;
end

%% Grid and cost

% Grid Bounds
gMin = [-5; -5; -pi; -5];
gMax = [ 5;  5;  pi; 5];

% createGrid takes in grid bounds, grid number, and periodic dimensions
sD.grid = createGrid(gMin, gMax, gN, 3);

% Cost Function shapeCylinder(grid,ignoreDims,center,radius);
%data0 = shapeCylinder(sD.grid,[3 4],[0;0],1);
data0 = shapeIntersection(shapeCylinder(sD.grid,[3 4],[0;0],1), ...
                          shapeRectangleByCorners(sD.grid, [-5 -5 -Inf -0.1], [5 5 +Inf 0.1]));
%data0 = shapeRectangleByCorners(sD.grid, [-1 -1 -Inf -0.1], [1 1 +Inf 0.1]);


if visualize
  % to visualize initial function over 2D grid, project onto 2D
  [g2D, data02D] = proj(sD.grid,data0,[0 0 1 1],[0 0]);
  
  figure(1)
  clf
  subplot(1,2,1)
  
  % plot data02D over grid in x and y
  surf(g2D.xs{1}, g2D.xs{2}, data02D)
  
  xlabel('$x_r$','Interpreter','latex','FontSize',20)
  ylabel('$y_r$','Interpreter','latex','FontSize',20)
  zlabel('Cost function','FontSize',20)
end

%% Dynamical system

% min and max turn rate for tracker
uMin = -8;
uMax = 8;

% min and max velocities for planner (in x and y)
pMin = [-.5; -.5];
pMax = [.5; .5];

% min and max disturbance for wind (in x and y)
dMax = [0; 0];
dMin = [0; 0];

% max and min acceleration for tracker
aMin = -5;
aMax = 5;

% number of dimensions in the system
dims = 1:4;

% create dynamic system
sD.dynSys = P4D_Q2D_Rel([], uMin, uMax, pMin, pMax, dMin, dMax, aMin, aMax, dims);

%% Other Parameters

% is tracker minimizing or maximizing?
sD.uMode = 'min';

% is planner minimizing or maximizing?
sD.dMode = 'max';

% how high accuracy?
sD.accuracy = 'low';

if visualize
  % set visualize to true
  extraArgs.visualize = true;
  
  % set slice of function to visualize
  extraArgs.RS_level = 0;
  
  % figure number
  extraArgs.fig_num = 2;

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
% extraArgs.stopConverge = true;

% converges when function doesn't change by more than dt each time step
% extraArgs.convergeThreshold = dt;

% solve backwards reachable tube
[data, tau] = HJIPDE_solve(data0, tau, sD, 'zero', extraArgs);

% take function at final time for full BRT
data = data(:,:,:,:,end);

if visualize
  % visualize function
  figure(1)
  subplot(1,2,2)
  [g2D, data2D] = proj(sD.grid, data, [0 0 1 1], [0 0]);
  surf(g2D.xs{1}, g2D.xs{2}, data2D)
  xlabel('$x_r$','Interpreter','latex','FontSize',20)
  ylabel('$y_r$','Interpreter','latex','FontSize',20)
  zlabel('Value function','FontSize',20)
  
  % visualize BRT
  figure(2)
  clf
  %alpha = .5;
  level = 0;
  
  h0 = visSetIm(sD.grid, data0, 'blue', level(1));

  hold on
  h = visSetIm(sD.grid, data, 'red', level(1));
  %h.FaceAlpha = alpha;
  axis square
  xlabel('$x_r$','Interpreter','latex','FontSize',20)
  ylabel('$y_r$','Interpreter','latex','FontSize',20)
  zlabel('$\theta$','Interpreter','latex','FontSize',20)
  
  set(gcf,'Color','white')
end

%tracking error bound
teb = sqrt(min(data(:)));
end

