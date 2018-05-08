function simulateOnlineFSM(g5D, deriv, obs_filename, extraArgs)
% simulateOnline_full(data_filename, obs_filename, extraArgs)
%     Includes tracking
%
%inputs:
%       gs           -  cell of grids
%       datas        -  cell of datas
%       quadTrue     -  obj of true vehicle, must have dynSys and dMax
%       quadVirt     -  obj of virtual vehicle, (needed for RRT?)
%       quadRel      -  obj of relative vehicle, must have dynSys
%       extraArgs    -  this structure can be used to leverage other
%                       additional functionalities within this function.
%                       Its subfields are:
%           .Q            -  Matrix to compare position states
%           .costFunction -  type of cost function, default quadratic
%           .environment  -  type of environment, default known
%           .dt           -  initial time step
%       inputs related to sensing environment
%       inputs related to RRT (goal?)

addpath(genpath('.'))

% if ~(exist('CCMotion') == 3)
  mex Planners/FSM/CCMotion.cpp
% end

warning off

% Video
vout = VideoWriter('figs/video.mp4', 'MPEG-4');
vout.Quality = 100;
vout.FrameRate = 10;
vout.open;

%% Problem setup
trackErr = 0.065; %%%%%%%
sense_range = 0.5;
sense_angle = pi/6;
sense_region = create_sensing_region(sense_range, sense_angle);

start = [-0.75; -0.75; pi/6];
goal = [0.5; 0.5; pi/2];
goal_size = [2.1*trackErr; 2.1*trackErr; pi/6];

dt = 0.1;

if nargin < 3
  obs_filename = 'Planners/FSM/obs.mat';
end

if nargin < 3
  extraArgs = [];
end

if ~isfield(extraArgs, 'visualize')
  vis = true;
end

%% Load files
% Preprocess look-up table for speed (since augmenting matrices is very
% slow in 5D)
[~, deriv{4}] = augmentPeriodicData(g5D, deriv{4});
[g5D, deriv{5}] = augmentPeriodicData(g5D, deriv{5});
g5D.bdry{3} = g5D.bdry{2};

load(obs_filename) % This also has a variable named g!

% Dynamical system
aRange = [-0.5 0.5];
alphaMax = 6;
vOther = 0.1;
wMax = 1.5;
dMax = [0.02 0.02 0.2 0.02];

dynSys = P5D_Dubins_Rel([], aRange, alphaMax, vOther, wMax, dMax);
  
uMode = 'max';

obsMap = ObstacleMapLS(g2D, obs2D);

% plot global obstacles
if vis
  f = figure;
  f.Color = 'white';
  f.Position = [100 100 1280 720];
  
  obsMap.plotGlobal()
  f.Children.FontSize = 16;
  hold on
  quiver(goal(1), goal(2), 0.1*cos(goal(3)), 0.1*sin(goal(3)), 'b')
  plotDisk(goal(1:2), goal_size(1), 'b-');
  
  xlabel('x', 'FontSize', 16)
  ylabel('y', 'FontSize', 16)
  
  axis equal
  xlim([-L L])

  box on
  grid on
end

% set initial states to zero
virt_x = start;

% Create real quadrotor system
trueCar = Plane5D([start; 0.1; 0], dynSys.alphaMax, dynSys.aRange, dynSys.dMax);

% % define when to switch from safety control to performance control
% small = 1;
% safetyBound = bubble - small.*[gs{1}.dx(1); gs(2).dx(1); gs(3).dx(1)];

%% Start loop! Tracking error bound block
%input: environment, sensing
%output: augmented obstacles
newStates = [];
iter = 0;
global_start = tic; % Time entire simulation

max_iter = 50000;
lookup_time = 0;
plan_time = 0;

% Smaller grid for planning
gs = createGrid(g.min, g.max, 31*ones(3,1), 3);
sensed_new_total = 0;
while iter < max_iter
  iter = iter + 1;

  % 1. Sense your environment, locate obstacles
  % 2. Expand sensed obstacles by tracking error bound
  sensed_new = obsMap.sense_update(trueCar.x(1:3), sense_region, trackErr);

  %% Path Planner Block
  % Replan if a new obstacle is seen
  
  if sensed_new
    sensed_new_total = sensed_new_total + 1;
  end
  
  plan_this_iter = isempty(newStates) || sensed_new_total >= 10;
  
  if plan_this_iter
    sensed_new_total = 0;
    
    % Update next virtual state
    obs3D = repmat(obsMap.padded_obs, [1 1 g.N(3)]);
    
    obs3Ds = migrateGrid(g, obs3D, gs);
    plan_start = tic;
    if iter == 1
      [newStates, uPlan] = fsmNextState(virt_x, goal, goal_size, L, gs, ...
        obs3Ds, dt, dynSys.vOther, dynSys.wMax, [], false);
    else
      [newStates, uPlan] = fsmNextState(virt_x, goal, goal_size, L, gs, ...
        obs3Ds, dt, dynSys.vOther, dynSys.wMax, uPlan, false);
    end
    plan_time = plan_time + toc(plan_start);
    
    if vis
      if iter == 1
        hPath = plot(newStates(1,:), newStates(2,:), ':');
      else
        hPath.XData = newStates(1,:);
        hPath.YData = newStates(2,:);
      end
    end    
  end
  
  virt_x = newStates(:,1);
  newStates(:,1) = [];

  %% Hybrid Tracking Controller
  look_up_start = tic;
  u = P5D_Dubins_htc(dynSys, uMode, trueCar.x, virt_x, g5D, deriv);
  lookup_time = lookup_time + toc(look_up_start);
  
  %% True System Block
  % 1. add random disturbance to velocity within given bound
  d = -dynSys.dMax + 2*rand(4,1).*dynSys.dMax;
  
  % 2. update state of true vehicle
  trueCar.updateState(u, dt, [], d);
  
  % Make sure error isn't too big (shouldn't happen)
  if norm(virt_x(1:2) - trueCar.x(1:2)) > 0.22
    keyboard
  end
  
  %% Virtual System Block  
%   fprintf('Iteration took %.2f seconds\n', toc);
  
  % Visualize
  if vis
    % Local obstacles and true position
    obsMap.plotLocal;
    obsMap.plotPadded;
    obsMap.plotSenseRegion;    
%     if iter == 1
%       quiver(start(1), start(2), 0.2*cos(start(3)), 0.2*sin(start(3)))
%       hold on
%     end
    
    % Virtual state
    if exist('hV', 'var')
      hV.XData = virt_x(1);
      hV.YData = virt_x(2);
      hV.UData = 0.1*cos(virt_x(3));
      hV.VData = 0.1*sin(virt_x(3));
      
      hV_true.XData = trueCar.x(1);
      hV_true.YData = trueCar.x(2);
      hV_true.UData = 0.1*cos(trueCar.x(3));
      hV_true.VData = 0.1*sin(trueCar.x(3));
    else
      hV = quiver(virt_x(1), virt_x(2), 0.1*cos(virt_x(3)), ...
        0.1*sin(virt_x(3)), '*', 'color', [0 0.75 0]);
      
      hV_true = quiver(trueCar.x(1), trueCar.x(2), 0.1*cos(trueCar.x(3)), ...
        0.1*sin(trueCar.x(3)));
    end
    
    title(sprintf("t = %.1f", (iter-1)*dt))
    
    drawnow

    export_fig(sprintf('figs/%d', iter), '-pdf')
    savefig(sprintf('figs/%d.fig', iter))
    
    current_frame = getframe(gcf); % gca does just the plot
    writeVideo(vout, current_frame);
  end
  
  % Check goal status
  reached_goal = 0;
  for k = 1:3
    reached_goal = reached_goal+((trueCar.x(k) - goal(k))/goal_size(k))^2;
  end
  
  if reached_goal <= 1.01
    break;
  end
  
end

% vout.close
comp_time = toc(global_start);
fprintf('=== TOTAL ===\n')
fprintf('%d iterations in %.4f seconds\n', iter, comp_time)
fprintf('%.4f seconds per iteration on average\n', comp_time/iter)

fprintf('=== PLANNING ===\n')
fprintf('%d iterations in %.4f seconds\n', iter, plan_time)
fprintf('%.4f seconds per iteration on average\n', plan_time/iter)

fprintf('=== TRACKING ===\n')
fprintf('%d iterations in %.4f seconds\n', iter, lookup_time)
fprintf('%.4f seconds per iteration on average\n', lookup_time/iter)


end
