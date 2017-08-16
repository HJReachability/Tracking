function [data,g_max,g_min,g_N]=Point_3D_3D_RS(N, vtracker, vplanner, visualize)

if nargin < 1
  N = 200;
end
g_N = N*ones(3,1);

if nargin <2
    vtracker = 1;
end

if nargin <3
    vplanner = .75;
end

if nargin < 4
  visualize = true;
end


%% Grid and cost

% Grid Bounds
g_min = -2*ones(3,1);
g_max = 2*ones(3,1);

% createGrid takes in grid bounds, grid number, and periodic dimensions
sD.grid = createGrid(g_min, g_max, g_N);

% Cost Function
data0 = sD.grid.xs{1}.^2 + sD.grid.xs{2}.^2 +sD.grid.xs{3}.^2;

% obstacles
%extraArgs.obstacles = -data0;  %%%%%%%%%%%%%%%%%%%%

if visualize
  % to visualize initial function over 2D grid, project onto 2D
  [g2D, data02D] = proj(sD.grid,data0,[0 0 1],0);
  
  figure(1)
  clf
  subplot(1,2,1)
  
  % plot data02D over grid in x and y
  surf(g2D.xs{1}, g2D.xs{2}, sqrt(data02D))
end

%% Dynamical system

% min and max speed for tracker
uMin = -vtracker*ones(3,1);
uMax = vtracker*ones(3,1);

% min and max speed for planner
pMin = -vplanner*ones(3,1);
pMax = vplanner*ones(3,1);

% number of dimensions in the system
dims = 1:3;

% create dynamic system
sD.dynSys = Point_3D_3D_Rel([], uMin, uMax, pMin, pMax, dims);

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
  extraArgs.RS_level = 2; 
  
  % figure number
  extraArgs.fig_num = 2;
  f = figure(2);
  extraArgs.plotData.plotDims = [1 1 0];
  extraArgs.plotData.projpt = 0;
  
  % delete previous time step's plot
  extraArgs.deleteLastPlot = false;
end

% time step
dt = 0.01;

% Max time
tMax = .75;

% Vector of times
tau = 0:dt:tMax;

% stop when function has converged
extraArgs.stopConverge = true;

% converges when function doesn't change by more than dt each time step
extraArgs.convergeThreshold = dt;

extraArgs.keepLast = 1;

% solve backwards reachable set
[data, tau] = HJIPDE_solve(data0, tau, sD, 'max_data0', extraArgs); 

% largest cost along all time (along the entire 5th dimension which is
% time)
%data = max(data,[],5); %%%%%%%%%%%%%%%%%%%%%%%%%%%

if visualize
  figure(1)
  subplot(1,2,2)
  [g2D, data2D] = proj(sD.grid, data, [0 0 1], 0);
  s = surf(g2D.xs{1}, g2D.xs{2}, sqrt(data2D));
  
  %Level set for tracking error bound
  lev1 = min(min(s.ZData));
  
  %tracking error bound
teb = sqrt(min(data(:))) + 0.05;

%lev = 0.4408;
lev = lev1 + 0.03;
  
  
  f = figure(2);
  clf
  alpha = .2;
  small = .05;
  
  levels = [lev1, lev, teb];  
  
  g3D = sD.grid;
  data3D = data;
  data03D = data0;
  
  for i = 1:3
      subplot(2,3,i)
      h0 = visSetIm(g3D, sqrt(data03D), 'blue', levels(i)+small);
      h0.FaceAlpha = alpha;
      hold on
      h = visSetIm(g3D, sqrt(data3D), 'red', levels(i));
      axis([-levels(3)-small levels(3)+small ...
        -levels(3)-small levels(3)+small -pi pi])
    if i == 2
      title(['t = ' num2str(tau(end)) ' s'])
    end
      axis square
  end
  
  for i = 4:6
      subplot(2,3,i)
      h0 = visSetIm(g2D, sqrt(data02D), 'blue', levels(i-3)+small);
      hold on
      h = visSetIm(g2D, sqrt(data2D), 'red', levels(i-3));
      axis([-levels(3)-small levels(3)+small ...
        -levels(3)-small levels(3)+small])
      title(['R = ' num2str(levels(i-3))])
      axis square
          
  set(gcf,'Color','white')
  end
end

