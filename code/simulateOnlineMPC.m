function simulateOnlineMPC(data_filename, obs_filename, extraArgs)
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

%% Problem setup (DEFINE YOUR OWN HERE)
% start = [-0.75; -0.75; pi/6];
% goal = [0.5; 0.5; pi/2];
% trackErr = 0.05; %%%%%%%
% sense_range = 0.5;
% sense_angle = pi/6;
% sense_region = create_sensing_region(sense_range, sense_angle);
% 
% virt_v = 0.5;
% 
% delta_x = virt_v*dt;

if nargin < 1
  data_filename = 'Q8D_Q4D_RS_1.00aMax.mat';
end

load(data_filename)
dt = tau(2) - tau(1);

% Initial list of tracking error bounds, now in ascending-time order
TEB_ind = length(tau);
TEB_list = flip(TEB(1:TEB_ind-1));
min_level = minData(end)*1.01;

if nargin < 2
  obs_filename = 'OBS_FILE_NAME_HERE.mat';
end

if nargin < 3
  extraArgs = [];
end

% matrix to compare position states (virt vs. true)
if ~isfield(extraArgs,'Q')
  Q = zeros(8,4);
  Q(1,1) = 1;
  Q(2,2) = 1;
  Q(5,3) = 1;
  Q(6,4) = 1;
end

if ~isfield(extraArgs, 'visualize')
  vis = true;
end

%% Before Looping
% load(data_filename)
load(obs_filename)

uMode = 'max';
% dMode = 'min'; % Not needed since we're not using worst-case control

% obsMap = ObstacleMapLS(g2D, obs2D);

% % plot global obstacles
% if vis
%   f = figure;
%   f.Color = 'white';
%   f.Position = [100 100 1280 720];
%   
%   obsMap.plotGlobal()
%   f.Children.FontSize = 16;
%   hold on
%   quiver(goal(1), goal(2), 0.2*cos(goal(3)), 0.2*sin(goal(3)), 'b')
%   plotDisk(goal(1:2), 0.2, 'b-');
%   
%   xlabel('x', 'FontSize', 16)
%   ylabel('y', 'FontSize', 16)
%   
%   axis equal
%   xlim([-L L])
% 
%   box on
%   grid on
% end

% % set initial states to zero
% true_x = zeros(5,1);
% true_x(1:3) = start;
virt_x = start;

% Create real quadrotor syste
dynSys = sD.dynSys;
trueQuad = Quad8D(start_x, dynSys.uMin, dynSys.uMax, dynSys.dMin, ...
  dynSys.dMax, 1:8);

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
%%%%%%%%%%%% MPC PLANNING BLOCK HERE %%%%%%%%%%
%   % 1. Sense your environment, locate obstacles
%   % 2. Expand sensed obstacles by tracking error bound
%   sensed_new = obsMap.sense_update(virt_x, sense_region, trackErr);
%   
%   %% Path Planner Block
%   % Replan if a new obstacle is seen
%   if isempty(newStates) || sensed_new
%     % Update next virtual state
%     obs3D = repmat(obsMap.padded_obs, [1 1 g.N(3)]);
%     newStates = fsmNextState(virt_x, goal, L, g, obs3D, delta_x, false);
%     
%     if iter == 1
%       hPath = plot(newStates(1,:), newStates(2,:), ':');
%     else
%       hPath.XData = newStates(1,:);
%       hPath.YData = newStates(2,:);
%     end
%   end
%   virt_x = newStates(:,1);
%   newStates(:,1) = [];

  %% Hybrid Tracking Controller
  % 1. find relative state
  local_start = tic;
  rel_x = trueQuad.x - Q*virt_x;
  
  % 2. Determine which controller to use, find optimal control
  % get spatial gradients
  XDims = 1:4;
  YDims = 5:8;
  
  deriv_TEB_ind = cell(4,1);
  for k = 1:4
    deriv_TEB_ind{k} = deriv{k}(:,:,:,:,TEB_ind);
  end
  pX = eval_u(g, deriv_TEB_ind, rel_x(XDims));
  pY = eval_u(g, deriv_TEB_ind, rel_x(YDims));

  % Find optimal control of relative system (no performance control)
  uX = dynSys.optCtrl([], rel_x(XDims), pX, uMode);
  uY = dynSys.optCtrl([], rel_x(YDims), pY, uMode);

  u = [uX uY];
  lookup_time = lookup_time + toc(local_start);
  
  %% True System Block
  % 1. add random disturbance to velocity within given bound
  d = dynSysX.dMin + rand(2,1).*(dynSysX.dMax - dynSysX.dMin);
  
  % 2. update state of true vehicle
  trueQuad.updateState(u, dt, [], d);

  
  %% Determine which tracking error bound to start with next (takes about 0.2s)
  
  TEB_ind_x = get_TEB_ind(tau, sD, data, rel_x(1:4), TEB, min_level)
  TEB_ind_y = get_TEB_ind(tau, sD, data, rel_x(5:8), TEB, min_level)
 
  TEB_ind = min(TEB_ind_x, TEB_ind_y);
  TEB_list = flip(TEB(1:TEB_ind-1));
  
  % Make sure error isn't too big (shouldn't happen)
  if norm(virt_x - trueQuad.x([1 2 5 6])) > 10 % Modify this bound
    keyboard
  end
  
  %% Virtual System Block  
%   fprintf('Iteration took %.2f seconds\n', toc);
  
%   % Visualize
%   if vis
%     % Local obstacles and true position
%     obsMap.plotLocal;
%     obsMap.plotPadded;
%     obsMap.plotSenseRegion;    
%     if iter == 1
%       quiver(start(1), start(2), 0.2*cos(start(3)), 0.2*sin(start(3)))
%       hold on
%     end
%     
%     % Virtual state
%     if exist('hV', 'var')
%       hV.XData = virt_x(1);
%       hV.YData = virt_x(2);
%       hV.UData = 0.2*cos(virt_x(3));
%       hV.VData = 0.2*sin(virt_x(3));
%     else
%       hV = quiver(virt_x(1), virt_x(2), 0.2*cos(virt_x(3)), ...
%         0.2*sin(virt_x(3)), '*', 'color', [0 0.75 0]);
%     end
%     
%     drawnow
% 
%     export_fig(sprintf('pics/%d', iter), '-png')
% %     savefig(sprintf('pics/%d.fig', iter))    
%   end
end

% comp_time = toc(global_start);
% fprintf('%d iterations in %.4f seconds\n', iter, comp_time)
% fprintf('%.4f seconds per iteration on average\n', comp_time/iter)
% 
% fprintf('%d iterations in %.4f seconds\n', iter, lookup_time)
% fprintf('%.4f seconds per iteration on average\n', lookup_time/iter)


end
