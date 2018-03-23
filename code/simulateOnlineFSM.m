function simulateOnlineFSM(data_filename, obs_filename, extraArgs)
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
mex Planners/FSM/CCMotion.cpp

%% Problem setup
start = [-0.75; -0.75; pi/6];
goal = [0.5; 0.5; pi/2];
trackErr = 0.05; %%%%%%%
sense_range = 0.5;
sense_angle = pi/6;
sense_region = create_sensing_region(sense_range, sense_angle);

virt_v = 0.5;

dt = 0.01;
delta_x = virt_v*dt;

if nargin < 1
  data_filename = 'P5D_Dubins_RS.mat';
end

if nargin < 2
  obs_filename = 'Planners/FSM/obs.mat';
end

if nargin < 3
  extraArgs = [];
end

% matrix to compare position states (virt vs. true)
if ~isfield(extraArgs,'Q')
  Q = zeros(8,2);
  Q(1,1) = 1;
  Q(5,2) = 1;
end

if ~isfield(extraArgs, 'visualize')
  vis = true;
end

%% Before Looping
load(data_filename)
load(obs_filename)

uMode = 'max';
% dMode = 'min'; % Not needed since we're not using worst-case control

obsMap = ObstacleMapLS(g2D, obs2D);

% plot global obstacles
if vis
  f = figure;
  f.Color = 'white';
  f.Position = [100 100 1280 720];
  
  obsMap.plotGlobal()
  f.Children.FontSize = 16;
  hold on
  quiver(goal(1), goal(2), 0.2*cos(goal(3)), 0.2*sin(goal(3)), 'b')
  plotDisk(goal(1:2), 0.2, 'b-');
  
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
trueCar = Plane5D([start; 0; 0], dynSys.alphaMax, dynSys.aRange, dynSys.dMax);

% % define when to switch from safety control to performance control
% small = 1;
% safetyBound = bubble - small.*[gs{1}.dx(1); gs(2).dx(1); gs(3).dx(1)];

%% Start loop! Tracking error bound block
%input: environment, sensing
%output: augmented obstacles
newStates = [];
iter = 0;
global_start = tic; % Time entire simulation

max_iter = 5000;
lookup_time = 0;

% while iter < max_iter && norm(trueQuad.x([1 5 9]) - goal) > 0.5
while iter < max_iter && norm(virt_x - goal) > 0.25
  iter = iter + 1;

  % 1. Sense your environment, locate obstacles
  % 2. Expand sensed obstacles by tracking error bound
  sensed_new = obsMap.sense_update(virt_x, sense_region, trackErr);
  
  %% Path Planner Block
  % Replan if a new obstacle is seen
  if isempty(newStates) || sensed_new
    % Update next virtual state
    obs3D = repmat(obsMap.padded_obs, [1 1 g.N(3)]);
    newStates = fsmNextState(virt_x, goal, L, g, obs3D, delta_x, false);
    
    if iter == 1
      hPath = plot(newStates(1,:), newStates(2,:), ':');
    else
      hPath.XData = newStates(1,:);
      hPath.YData = newStates(2,:);
    end
  end
  virt_x = newStates(:,1);
  newStates(:,1) = [];

  %% Hybrid Tracking Controller
  u = P5D_Dubins_htc(sD.dynSys, uMode, true_x, virt_x, g, deriv);
  lookup_time = lookup_time + toc(local_start);
  
  %% True System Block
  % 1. add random disturbance to velocity within given bound
  d = -dynSys.dMax + 2*rand(4,1).*dynSys.dMax;
  
  % 2. update state of true vehicle
  trueCar.updateState(u, dt, [], d);
  
  % Make sure error isn't too big (shouldn't happen)
  if norm(virt_x(1:2) - trueCar.x(1:2)) > 3
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
    if iter == 1
      quiver(start(1), start(2), 0.2*cos(start(3)), 0.2*sin(start(3)))
      hold on
    end
    
    % Virtual state
    if exist('hV', 'var')
      hV.XData = virt_x(1);
      hV.YData = virt_x(2);
      hV.UData = 0.2*cos(virt_x(3));
      hV.VData = 0.2*sin(virt_x(3));
    else
      hV = quiver(virt_x(1), virt_x(2), 0.2*cos(virt_x(3)), ...
        0.2*sin(virt_x(3)), '*', 'color', [0 0.75 0]);
    end
    
    drawnow

    export_fig(sprintf('pics/%d', iter), '-png')
%     savefig(sprintf('pics/%d.fig', iter))    
  end
end

% comp_time = toc(global_start);
% fprintf('%d iterations in %.4f seconds\n', iter, comp_time)
% fprintf('%.4f seconds per iteration on average\n', comp_time/iter)
% 
% fprintf('%d iterations in %.4f seconds\n', iter, lookup_time)
% fprintf('%.4f seconds per iteration on average\n', lookup_time/iter)


end
