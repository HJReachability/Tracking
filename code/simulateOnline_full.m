function simulateOnline_full(data_filename, obs_filename, extraArgs)
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

% Problem setup
start = [-12; 0; 0];
goal = [12; 0; 0];
trackErr = 0.8;
senseRange = 2;

virt_v = 0.5;

dt = 0.1;
delta_x = virt_v*dt;

% Subsystems
XDims = 1:4;
YDims = 5:8;
ZDims = 9:10;

if nargin < 1
  data_filename = 'Quad10D_g61_dt01_t50_veryHigh_quadratic.mat';
end

if nargin < 2
  obs_filename = 'obs.mat';
end

if nargin < 3
  extraArgs = [];
end

% matrix to compare position states (virt vs. true)
if ~isfield(extraArgs,'Q')
  Q = zeros(10,3);
  Q(1,1) = 1;
  Q(5,2) = 1;
  Q(9,3) = 1;
end

if ~isfield(extraArgs, 'visualize')
  vis = true;
end

%% Before Looping
load(data_filename)
load(obs_filename)

uMode = 'max';
% dMode = 'min'; % Not needed since we're not using worst-case control

obsMap = ObstacleMap(obs);

% plot global obstacles
if vis
  f = figure;
  f.Color = 'white';
  f.Position = [100 100 1280 720];
  
  obsMap.plotGlobal()
  f.Children.FontSize = 16;
  hold on
  plot3(goal(1), goal(2), goal(3), 'bo')
  
  xlabel('x', 'FontSize', 16)
  ylabel('y', 'FontSize', 16)
  zlabel('z', 'FontSize', 16)
  
  axis equal
  xlim([-15 15])
%   
%   ylim([-10 10])
%   zlim([-10 10])
  view([-10 10])
end

% set initial states to zero
true_x = zeros(10,1);
true_x([1 5 9]) = start;

% Create real quadrotor system
rl_ui = [2 4 6];
trueQuad = Quad10D(true_x, dynSysX.uMin(rl_ui), dynSysX.uMax(rl_ui), ...
  dynSysX.dMin, dynSysX.dMax, 1:10);

% % define when to switch from safety control to performance control
% small = 1;
% safetyBound = bubble - small.*[gs{1}.dx(1); gs(2).dx(1); gs(3).dx(1)];

%% Start loop! Tracking error bound block
%input: environment, sensing
%output: augmented obstacles
newStates = [];
iter = 0;
while norm(true_x([1 5 9]) - goal) > 1
  tic
  iter = iter + 1;

  % 1. Sense your environment, locate obstacles
  % 2. Expand sensed obstacles by tracking error bound
  sensed_new = obsMap.sense_update(true_x([1 5 9]), senseRange, trackErr);
  
  %% Path Planner Block
  % Replan if a new obstacle is seen
  if isempty(newStates) || sensed_new
    % Update next virtual state
    newStates = rrtNextState(true_x([1 5 9]), goal, obsMap.padded_obs, ...
      delta_x, [], false);
  end
  virt_x = newStates(1,:)';
  newStates(1,:) = [];

  %% Hybrid Tracking Controller
  % 1. find relative state
  rel_x = true_x - Q*virt_x;
  
  % 2. Determine which controller to use, find optimal control
  %get spatial gradients
  pX = eval_u(gX, derivX, rel_x(XDims));
  pY = eval_u(gX, derivX, rel_x(YDims));
  pZ = eval_u(gZ, derivZ, rel_x(ZDims));
  
  % Find optimal control of relative system (no performance control)
  uX = dynSysX.optCtrl([], rel_x(XDims), pX, uMode);
  uY = dynSysX.optCtrl([], rel_x(YDims), pY, uMode);
  uZ = dynSysZ.optCtrl([], rel_x(ZDims), pZ, uMode);
  u = [uX uY uZ];
  u = u(rl_ui);
  
  %% True System Block
  % 1. add random disturbance to velocity within given bound
  d = dynSysX.dMin + rand(3,1).*(dynSysX.dMax - dynSysX.dMin);
  
  % 2. update state of true vehicle
  trueQuad.updateState(u, dt, [], d);

  true_x = trueQuad.x;

  % Make sure error isn't too big (shouldn't happen)
  if norm(virt_x - true_x([1 5 9])) > 3
    keyboard
  end
  
  %% Virtual System Block  
  fprintf('Iteration took %.2f seconds\n', toc);
  
  % Visualize
  if vis
    % Local obstacles and true position
    obsMap.plotLocal;
    plot3(true_x(1), true_x(5), true_x(9), 'b.')
%     trueQuad.plotPosition();
    hold on
    
    % Virtual state
    if exist('hV', 'var')
      hV.XData = virt_x(1);
      hV.YData = virt_x(2);
      hV.ZData = virt_x(3);
    else
      hV = plot3(virt_x(1), virt_x(2), virt_x(3), '*', 'color', [0 0.5 0]);
    end
    
    % Tracking error bound
    left_bd = virt_x(1) - trackErr;
    right_bd = virt_x(1) + trackErr;
    back_bd = virt_x(2) - trackErr;
    front_bd = virt_x(2) + trackErr;
    bottom_bd = virt_x(3) - trackErr;
    top_bd = virt_x(3) + trackErr;
    
    left_surf = [left_bd, back_bd, bottom_bd; ...
      left_bd, back_bd, top_bd; ...
      left_bd, front_bd, top_bd; ...
      left_bd, front_bd, bottom_bd];
    
    right_surf = [right_bd, back_bd, bottom_bd; ...
      right_bd, back_bd, top_bd; ...
      right_bd, front_bd, top_bd; ...
      right_bd, front_bd, bottom_bd];
    
    back_surf = [left_bd, back_bd, bottom_bd; ...
      left_bd, back_bd, top_bd; ...
      right_bd, back_bd, top_bd; ...
      right_bd, back_bd, bottom_bd];    
 
    front_surf = [left_bd, front_bd, bottom_bd; ...
      left_bd, front_bd, top_bd; ...
      right_bd, front_bd, top_bd; ...
      right_bd, front_bd, bottom_bd];
    
    bottom_surf = [left_bd, front_bd, bottom_bd; ...
      left_bd, back_bd, bottom_bd; ...
      right_bd, back_bd, bottom_bd; ...
      right_bd, front_bd, bottom_bd];     
    
    top_surf = [left_bd, front_bd, top_bd; ...
      left_bd, back_bd, top_bd; ...
      right_bd, back_bd, top_bd; ...
      right_bd, front_bd, top_bd];         
    
    boxShape = cat(3, left_surf, right_surf);
    boxShape = cat(3, boxShape, back_surf);
    boxShape = cat(3, boxShape, front_surf);
    boxShape = cat(3, boxShape, bottom_surf);
    boxShape = cat(3, boxShape, top_surf);
    
    if iter == 1
      boxMap = ObstacleMap(boxShape);
    else
      boxMap.global_obs = boxShape;
    end
    boxMap.plotGlobal('b', '-');

  end

  drawnow
  
  if mod(iter, 5) == 1
    export_fig(sprintf('pics/%d', iter), '-png')
    savefig(sprintf('pics/%d.fig', iter))
  end

end
