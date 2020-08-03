function simulateOnline(data_filename, obs_filename, extraArgs)
% simulateOnline(data_filename, obs_filename, extraArgs)
%     Does not include tracking
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
start = [-10; 0; 0];
goal = [10; 0; 0];
delta_x = 0.25;
trackErr = 0.5;
senseRange = 1.5;
dt = 0.01;

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

obsMap = ObstacleMapRRT(obs);

% plot global obstacles
if vis
  figure
  obsMap.plotGlobal()
  hold on
end

% set initial states to zero
true_x = zeros(10,1);
true_x([1 5 9]) = start;
virt_x = start;
rel_x = true_x - Q*virt_x;

% % Create real quadrotor system
% rl_ui = [2 4 6];
% trueQuad = Quad10D(true_x, dynSysX.uMin(rl_ui), dynSysX.uMax(rl_ui), ...
%   dynSysX.dMin, dynSysX.dMax, 1:10);

% define when to switch from safety control to performance control
% small = 1;
% safetyBound = bubble - small.*[gs{1}.dx(1); gs(2).dx(1); gs(3).dx(1)];

%% Start loop! Tracking error bound block
%input: environment, sensing
%output: augmented obstacles
newStates = [];
iter = 0;
while norm(virt_x - goal) > 1
  tic
  iter = iter + 1;
  % 1. Sense your environment, locate obstacles
  % 2. Expand sensed obstacles by tracking error bound
  sensed_new = obsMap.sense_update(virt_x, senseRange, trackErr); 
  
  %% Path Planner Block
  %inputs: virtual state, augmented obstacles
  %outputs: desired virtual state
  
  % 1. run RRT stuff, get new virtual state. using dummy example for now.
  if isempty(newStates) || sensed_new
    newStates = rrtNextState(virt_x, goal, obsMap.padded_obs, delta_x, [], false);
  end
  virt_x = newStates(1,:)';
  newStates(1,:) = [];
  
%   %% Hybrid Tracking Controller
%   %inputs: desired virtual state, true state
%   %outputs: control
%   
%   % 1. find relative state
%   rel_x = true_x - Q*virt_x;
%   
%   % 2. Determine which controller to use, find optimal control
%   
%   %get spatial gradients
%   pX = eval_u(gX, derivX, rel_x(XDims));
%   pY = eval_u(gY, derivY, rel_x(YDims));
%   pZ = eval_u(gZ, derivZ, rel_x(ZDims));
%   
%   %if gradient is flat, use performance control
%   if any(p==0)
%     % 3a. use performance control
%     %Find optimal control of relative system
%     uX = dynSysX.optCtrl([], rel_x(XDims), pX, uMode);
%     uY = dynSysX.optCtrl([], rel_x(YDims), pY, uMode);
%     uZ = dynSysZ.optCtrl([], rel_x(ZDims), pZ, uMode);
%     
%   else
%     % 3b. use safety control
%     %Find optimal control of relative system
%     uX = dynSysX.optCtrl([], rel_x(XDims), pX, uMode);
%     uY = dynSysX.optCtrl([], rel_x(YDims), pY, uMode);
%     uZ = dynSysZ.optCtrl([], rel_x(ZDims), pZ, uMode);
%     
%   end
%   
%   u = [uX(rl_ui); uY(rl_ui); uZ(rl_ui)];
%   
%   %% True System Block
%   %inputs: control
%   %outputs: true system state
%   
%   % 1. add random disturbance to velocity within given bound
%   d = dynSysX.dMin + rand(3,1).*(dynSysX.dMax - dynSysX.dMin);
%   
%   % 2. update state of true vehicle
%   trueQuad.updateState(u, dt, [], d);
%   trueQuad.x([1 5 9]) = virt_x;
  true_x = virt_x;
  
  %% Virtual System Block
  % inputs: true system state
  % outputs: virtual system state
  
%   % 1. set virtual state to position states from true vehicle
%   virt_x = true_x([1 5 9]);
  
  % 2. check if reached goal. dummy equation for now
  
  fprintf('Iteration took %.2f seconds\n', toc);
  
  if vis
    obsMap.plotLocal;
%     obsMap.plotPadded;
    plot3(virt_x(1), virt_x(2), virt_x(3), '.')

  % plot local obstacles
  % plot expanded obstacles
  % plot planned path
  % plot current trajectory
  % 
  end
  
  drawnow
%   export_fig(sprintf('pics/%d', iter), '-png')
end
