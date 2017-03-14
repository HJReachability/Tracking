function timesteps = simulateOnline(gs,datas,quadTrue,quadVirt,quadRel,extraArgs)

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


%outputs:
%       timeSteps?    -  list of time steps for loop?

%% Before Looping

%matrix to compare position states (virt vs. true)
if ~isfield(extraArgs,'Q')
  Q = zeros(10,3);
  Q(1,1) = 1;
  Q(5,2) = 1;
  Q(9,3) = 1;
end

%set initial states to zero
quadTrue.x = zeros(10,1);
quadVirt.x = zeros(3,1);
quadRel.x = quadTrue.x - Q*quadVirt.x;

%find corresponding tracking error bound
if isfield(extraArgs, 'costFunction') && strcmp(costFunction,'quadratic')
  bubble = sqrt(-eval_u(gs,datas,quadRel.x));
else
  error('not set for other costFunctions!')
end

%expand obstacles by tracking error bound
if isfield(extraArgs, 'environment') && strcmp(environment,'known')
  %expand all obstacles by tracking error bound
end

%set safety look-up tables in x,y,z dimensions
derivs = cell(3,1);
for i = 1:3
  derivs{i} = computeGradients(gs{i}, datas{i});
end

%set initial time step
dtNew = dt;

% %define when to switch from safety control to performance control
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
quadVirt.x = quadVirt.x + [.1; .1; .1];

%% Hybrid Tracking Controller
%inputs: desired virtual state, true state
%outputs: control

% 1. find relative state
quadRel.x = quadTrue.x - Q*quadVirt.x;

% 2. Determine which controller to use, find optimal control

%get spatial gradients
p = eval_u(gs,derivs,quadRel.x);

%if gradient is flat, use performance control
if any(p==0)
  % 3a. use performance control
  
  %Find optimal control of relative system
  uOpt = quadRel.optCtrl(dt,body.x,p,uMode);
  
  %keep optimal control for true vehicle
  uTrue = {uOpt{4}, uOpt{8}, uOpt{10}};
else
  % 3b. use safety control
  
  %Find optimal control of relative system
  uOpt = quadRel.optCtrl(dt,body.x,p,uMode);
  
  %keep optimal control for true vehicle
  uTrue = {uOpt{4}, uOpt{8}, uOpt{10}};
end

%% True System Block
%inputs: control
%outputs: true system state

% 1. add random disturbance to velocity within given bound
d = (rand(3,1).*2-1).*quadTrue.dMax;

% 2. update state of true vehicle
quadTrue.updateState(uTrue, dtNew, [], d);

%% Virtual System Block
% inputs: true system state
% outputs: virtual system state

% 1. set virtual state to position states from true vehicle
quadVirt.x = quadTrue.x([1 5 9]);

% 2. check if reached goal. dummy equation for now
value = value + [.1; .1; .1];


dtNew = toc;
end
