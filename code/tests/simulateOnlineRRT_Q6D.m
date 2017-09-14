function simulateOnlineRRT_Q6D(planner_speed, data_filename, obs_filename, extraArgs)
% simulateOnlineRRT(data_filename, obs_filename, extraArgs)
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

workingDirectory = pwd;

% Problem setup
start = [-12; 0; 0];
goal = [12; 0; 0];


% Subsystems
dims = 1:6;
subDims = {[1 4], [2 5], [3 6]};

if nargin <1
    planner_speed = 1;
end

if nargin < 2
  data_filename = [workingDirectory '/planner_RRT3D_Matlab/speed_' ...
      num2str(planner_speed*10) '_tenths.mat'];
end

if nargin < 3
  obs_filename = 'obs.mat';
end

if nargin < 4
  extraArgs = [];
end


if ~isfield(extraArgs, 'visualize')
  vis = true;
else
  vis = true;
end

%% Before Looping
load(data_filename)
load(obs_filename)

for ii = 1:length(sD)
derivs{ii} = computeGradients(sD{ii}.grid,datas{ii},@upwindFirstFirst);
end

posDims = sD{1}.dynSys.pdim;
velDims = sD{1}.dynSys.vdim;

trackErr = max(trackingErrorBound);
virt_v = sD{1}.dynSys.pMax(1);
dt = 0.1;
delta_x = virt_v*dt;
senseRange = 2*trackErr+delta_x;

% matrix to compare position states (virt vs. true)
  Q = zeros(length(dims),3);
  Q(1,1) = 1;
  Q(2,2) = 1;
  Q(3,3) = 1;
  
% planner velocity in tracker's vdims
  Plan_v = zeros(length(dims),1);
  
uMode = 'min';
% dMode = 'min'; % Not needed since we're not using worst-case control

obsMap = ObstacleMapRRT(obs);

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
  box on
  grid on
end

% set initial states to zero
start_x = zeros(6,1);
start_x(posDims) = start;

% Create real quadrotor system
%rl_ui = [2 4 6];
trueQuad = Q6D(start_x, sD{1}.dynSys.uMin, sD{1}.dynSys.uMax, ...
  sD{1}.dynSys.dMin, sD{1}.dynSys.dMax, 1:6);

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

virt_x = [start];

while iter < max_iter && norm(trueQuad.x(posDims) - goal) > 0.5
  iter = iter + 1;
  virt_x_last = virt_x;

  % 1. Sense your environment, locate obstacles
  % 2. Expand sensed obstacles by tracking error bound
  sensed_new = obsMap.sense_update(trueQuad.x(posDims), senseRange, trackErr);
  
  %% Path Planner Block
  % Replan if a new obstacle is seen
  if isempty(newStates) || sensed_new
    % Update next virtual state
%     newStates = rrtNextState(trueQuad.x(posDims), goal, obsMap.padded_obs, ...
%       delta_x, [], false);
  newStates = rrtNextState(virt_x, goal, obsMap.padded_obs, ...
      delta_x, [], false);
  end
  virt_x = newStates(1,:)';
  delta_virt_x = virt_x - virt_x_last;
  planner_vel = delta_virt_x/dt;
  %planner_vel = (delta_virt_x > 0)*virt_v + (delta_virt_x <0)*(-virt_v) ...
  %    + (delta_virt_x == 0)*0;
  Plan_v(velDims) = planner_vel;
  newStates(1,:) = [];

  %% Hybrid Tracking Controller
  % 1. find relative state
  local_start = tic;
  rel_x = trueQuad.x - [Q*virt_x + Plan_v];
  %rel_x = trueQuad.x - Q*virt_x;
  
  % 2. Determine which controller to use, find optimal control
  %get spatial gradients
  for ii = 1:length(sD)
      p{ii} = eval_u(sD{ii}.grid, derivs{ii}, rel_x(subDims{ii}));
  end
%   pX = eval_u(gX, derivs{1}, rel_x(XDims));
%   pY = eval_u(gX, derivs{2}, rel_x(YDims));
%   pZ = eval_u(gZ, derivs{3}, rel_x(ZDims));
  
  % Find optimal control of relative system (no performance control)
  u = [];
  for ii = 1:length(sD)
      utemp = sD{ii}.dynSys.optCtrl([], rel_x(subDims{ii}), p{ii}, uMode);
      u = [u; utemp];
  end
  u = cell2mat(u);
%   uX = dynSysX.optCtrl([], rel_x(XDims), pX, uMode);
%   uY = dynSysX.optCtrl([], rel_x(YDims), pY, uMode);
%   uZ = dynSysZ.optCtrl([], rel_x(ZDims), pZ, uMode);
%  u = [uX uY uZ];
%  u = u(rl_ui);
  lookup_time = lookup_time + toc(local_start);
  
  %% True System Block
  % 1. add random disturbance to velocity within given bound
  %d = sD{ii}.dynSys.dMin + rand(6,1)...
  %    .*(sD{ii}.dynSys.dMax - sD{ii}.dynSys.dMin);
  %d = d./2;
  %d = dynSysX.dMin + rand(3,1).*(dynSysX.dMax - dynSysX.dMin);
  d = zeros(6,1);
  
  % 2. update state of true vehicle
  trueQuad.updateState(u, dt, [], d);

  % Make sure error isn't too big (shouldn't happen)
  if norm(virt_x - trueQuad.x(posDims)) > trackErr
    keyboard
  end
  
  %% Virtual System Block  
%   fprintf('Iteration took %.2f seconds\n', toc);
  
  % Visualize
  if vis
    % Local obstacles and true position
    obsMap.plotLocal;
    plot3(trueQuad.x(1), trueQuad.x(2), trueQuad.x(3), 'b.')
    hold on
    
    % Virtual state
    if exist('hV', 'var')
      hV.XData = virt_x(1);
      hV.YData = virt_x(2);
      hV.ZData = virt_x(3);
    else
      hV = plot3(virt_x(1,end), virt_x(2,end), virt_x(3,end), '*', 'color', [0 0.75 0]);
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
      boxMap = ObstacleMapRRT(boxShape);
    else
      boxMap.global_obs = boxShape;
    end
    boxMap.plotGlobal('b', '-');

    drawnow

%     export_fig(sprintf('pics/%d', iter), '-png')
%     savefig(sprintf('pics/%d.fig', iter))    
  end
end

comp_time = toc(global_start);
fprintf('%d iterations in %.4f seconds\n', iter, comp_time)
fprintf('%.4f seconds per iteration on average\n', comp_time/iter)

fprintf('%d iterations in %.4f seconds\n', iter, lookup_time)
fprintf('%.4f seconds per iteration on average\n', lookup_time/iter)


end
