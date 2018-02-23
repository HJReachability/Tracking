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
if ~isfield(extraArgs, 'visualize')
  vis = true;
end

%% Before Looping
% load(data_filename)
load(obs_filename)

% Create real quadrotor syste
dynSys = sD.dynSys;
trueQuad = Quad8D(start_x, dynSys.uMin, dynSys.uMax, dynSys.dMin, ...
  dynSys.dMax, 1:8);

virt_x = start_x([1 2 5 6]);
rel_x = trueQuad.x - Q*virt_x;

% % define when to switch from safety control to performance control
% small = 1;
% safetyBound = bubble - small.*[gs{1}.dx(1); gs(2).dx(1); gs(3).dx(1)];

%% Start loop! Tracking error bound block
%input: environment, sensing
%output: augmented obstacles

iter = 0;
global_start = tic; % Time entire simulation

max_iter = 5000;
lookup_time = 0;

% while iter < max_iter && norm(trueQuad.x([1 5 9]) - goal) > 0.5
while iter < max_iter && norm(virt_x - goal) > 0.25
  iter = iter + 1;
  
  %%%%%%%%%%%% MPC PLANNING BLOCK HERE %%%%%%%%%%
  p = nextState; %%%%%%%%%%%%%%
  
  %% Hybrid Tracking Controller  
  % 2. Determine which controller to use, find optimal control
  % get spatial gradients
  
  local_start = tic;
  
  u = Q8D_Q4D_htc(rel_sys, trueQuad.x, nextState, sD.grid, deriv, indX, indY);
  
  lookup_time = lookup_time + toc(local_start);
  
  %% True System Block
  % 1. add random disturbance to velocity within given bound
  d = dynSysX.dMin + rand(2,1).*(dynSysX.dMax - dynSysX.dMin);
  
  % 2. update state of true vehicle
  trueQuad.updateState(u, dt, [], d);

  %% Determine which tracking error bound to start with next (takes about 0.2s)
  [indX, indY, TEB_list] = ...
    Q8D_Q4D_gti(trueQuad.x, nextState, sD.grid, data, level);
  
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
