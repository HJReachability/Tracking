function timesteps = simulateOnline(gs, datas, quadTrue, quadVirt, quadRel, ...
  data_filename, extraArgs)

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

if nargin < 7
  data_filename = 'Quad10D_g61_dt01_t50_veryHigh_quadratic.mat';
end

if nargin < 8
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
uMode = 'max';
% dMode = 'min'; % Not needed since we're not using worst-case control
dt = 0.1;
delta_x = dt*velocity;

% plot global obstacles
if vis
  plotGlobal = true;
  plotLocal = false;
  plotPadded = false;
  hG = rrt.obsmap.ObstaclePlot(plotGlobal, plotLocal, plotPadded);
end

% set initial states to zero
true_x = zeros(10,1);
virt_x = zeros(3,1);
rel_x = true_x - Q*virt_x;

% Create real quadrotor system
rl_ui = [2 4 6];
trueQuad = Quad10D(true_x, dynSysX.uMin(rl_ui), dynSysX.uMax(rl_ui), ...
  dynSysX.dMin, dynSysX.dMax, 1:10);

%find corresponding tracking error bound
if isfield(extraArgs, 'costFunction') && strcmp(costFunction,'quadratic')
  bubble = sqrt(-eval_u(gs, datas, rel_x));
else
  error('not set for other costFunctions!')
end

% expand obstacles by tracking error bound
if isfield(extraArgs, 'environment') && strcmp(environment,'known')
  % expand all obstacles by tracking error bound
end

% define when to switch from safety control to performance control
% small = 1;
% safetyBound = bubble - small.*[gs{1}.dx(1); gs(2).dx(1); gs(3).dx(1)];

%% Start loop! Tracking error bound block
%input: environment, sensing
%output: augmented obstacles
while value < goal
  tic
  if isfield(extraArgs,'environment') && strcmp(environment,'unknown')
    % 1. Sense your environment, locate obstacles
    
    % 2. Expand sensed obstacles by tracking error bound
  end
  
  %% Path Planner Block
  %inputs: virtual state, augmented obstacles
  %outputs: desired virtual state
  
  % 1. run RRT stuff, get new virtual state. using dummy example for now.
   [virt_x, rrt] = rrtNextState(delta_x, virt_x);
  
  %% Hybrid Tracking Controller
  %inputs: desired virtual state, true state
  %outputs: control
  
  % 1. find relative state
  rel_x = true_x - Q*virt_x;
  
  % 2. Determine which controller to use, find optimal control
  
  %get spatial gradients
  pX = eval_u(gX, derivX, rel_x(XDims));
  pY = eval_u(gY, derivY, rel_x(YDims));
  pZ = eval_u(gZ, derivZ, rel_x(ZDims));
  
  %if gradient is flat, use performance control
  if any(p==0)
    % 3a. use performance control
    %Find optimal control of relative system
    uX = dynSysX.optCtrl([], rel_x(XDims), pX, uMode);
    uY = dynSysX.optCtrl([], rel_x(YDims), pY, uMode);
    uZ = dynSysZ.optCtrl([], rel_x(ZDims), pZ, uMode);
    
  else
    % 3b. use safety control
    %Find optimal control of relative system
    uX = dynSysX.optCtrl([], rel_x(XDims), pX, uMode);
    uY = dynSysX.optCtrl([], rel_x(YDims), pY, uMode);
    uZ = dynSysZ.optCtrl([], rel_x(ZDims), pZ, uMode);
    
  end
  
  u = [uX(rl_ui); uY(rl_ui); uZ(rl_ui)];
  
  %% True System Block
  %inputs: control
  %outputs: true system state
  
  % 1. add random disturbance to velocity within given bound
  d = dynSysX.dMin + rand(3,1).*(dynSysX.dMax - dynSysX.dMin);
  
  % 2. update state of true vehicle
  trueQuad.updateState(u, dt, [], d);
  true_x = trueQuad.x;
  
  %% Virtual System Block
  % inputs: true system state
  % outputs: virtual system state
  
  % 1. set virtual state to position states from true vehicle
  virt_x = true_x([1 5 9]);
  
  % 2. check if reached goal. dummy equation for now
  
  fprintf('Iteration took %.2f seconds', toc);
  
  if vis
  % plot local obstacles
  % plot expanded obstacles
  % plot planned path
  % plot current trajectory
  % 
  end
end
