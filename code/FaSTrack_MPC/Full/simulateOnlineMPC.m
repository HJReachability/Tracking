%% FaSTrack+MPC (Full version)

clear;
close all; clc
addpath('./dynSys') % IMPORTANT: add this path BEFORE loading Q8D_Q4D_cpp_trunc.mat
addpath('./util')
addpath('./util/MPC_ACADO_based')

load Q8D_Q4D_cpp_trunc % pre-computed TEB and optimal control lookup table
load vOb % obstacle profile

if size(TEB,1) > 1
    TEB = TEB(2,:);
end

%% Problem setup
start = [2.0; 0.0];     % Start position
goal = [9.0; 11.5];     % Target position
goalTol = 1.0;          % Target reaching tolerance
sense_range = 6.0;      % Sensing range
resolution = 0.25;      % Map resolution
trackErr = 1.14;        % TEB on positional states
dt = 0.1;               % Duration per each tracking loop
Nmpc = 8;               % MPC prediction horizon
Tmpc = 0.2;             % MPC sampling time
iterMax = 5;            % Iterations in between two replan loops
PlantL = 0.2;           % Radius of the quadrotor
max_iter = 5000;        % Max iteration of the simulation process
vis = true;             % Visualization flag
uMode = 'max';          % Tracker mode


%% Initialization
% initialize vectors
virtStates   = [];
virtControls = [];
trueStates   = [];
trueControls = [];
runtime      = [];
lookup_time  = [];
interval_Ts  = [];
simTime      = 0;
simTotalTime = 0;
simTimeSpan  = [];
TrackingError = [];
MPC = {};

% initial list of tracking error bounds, now in ascending-time order
TEB_ind = length(tau);
TEB_list = flip(TEB(1:TEB_ind-1));
TEB_cell = {};
TEB_cell{end+1} = TEB_list;
min_level = level;
Q = zeros(8,4);
Q(1,1) = 1;
Q(2,2) = 1;
Q(5,3) = 1;
Q(6,4) = 1;

% create map object
obsMap = ObstacleMapMPC(vOb, resolution);

% initialize the virtual system
virtPos = start;                        % position of the virtual system: [x;y] 
virtQuad = [start(1);0;start(2);0];     % full state vector of the virtual system: [x;vx;y;vy] 
virtStates = [virtStates; virtQuad'];   % store the first virtual state

% initialize the real system
truePos = start;
dynSys = sD.dynSys;
trueQuad = Quad8D([start(1);0;0;0;start(2);0;0;0], ...
    dynSys.uMin, dynSys.uMax, dynSys.dMin, dynSys.dMax, 1:8);

% plot global obstacles
if vis
  f = figure;
  f.Color = 'white';
  f.Position = [660 660 665 665];
  obsMap.plotGlobal() 
  plotDisk(goal(1:2), goalTol, 'b-');
end

% define plot handles
hV = plotDisk(truePos, PlantL, 'r-');
hf = [];
hd = [];


%% Main loop
iter = 0;
EndFlag = true;
fprintf('Starting...\n')

while iter < max_iter && norm(truePos - goal) > goalTol
  %------ Map update ------
  % 1. sense your environment, locate obstacles
  % 2. augment sensed obstacles with the TEB
  obsMap.sense_update(truePos, sense_range, trackErr)
  
  if vis
    % plot the current position of real system
    delete(hV);
    hV = plotDisk([trueQuad.x(1);trueQuad.x(5)], PlantL, 'r-');  
      
    % plot updated obstacles
    obsMap.plotLocal;
    obsMap.plotPadded;
    obsMap.plotSenseRegion(truePos,sense_range);
  end
  
  %------ Replan using MPC ------
  if (isempty(virtStates) || mod(iter,iterMax) == 0) || EndFlag
    simTime = 0;
    if iter ~= 0
        delete(mpc.hT)            
        delete(hf)  
    end
    % update next virtual state    
    mpc = mpcNextState(virtQuad, goal, Nmpc, Nmpc, Tmpc, obsMap.vOb, iter*dt, TEB_list, tau(2)-tau(1));
    MPC{end+1} = mpc;
    for i = 1:length(mpc.stateOpt(:,1))
        mpc.stateOpt(i,1) = (mpc.stateOpt(i,1))*mpc.stateOpt(1,6)*Tmpc;
    end
    interval_Ts = [interval_Ts mpc.stateOpt(1,6)];
    
    % report runtime
    fprintf('\n******************************************\n');
    fprintf('***     MPC Path Planning Complete     ***\n');
    fprintf('******************************************\n');        
    fprintf('***   Total Runtime: %f s   ***\n', mpc.runtimeP2P+mpc.runtimeIteWS+mpc.runtimeOpt);
    fprintf('******************************************\n');
    fprintf('Point-to-point: %f s\n',mpc.runtimeP2P);
    fprintf('Iterative warmstart: %f s\n',mpc.runtimeIteWS);
    fprintf('Final Optimization: %f s\n',mpc.runtimeOpt);
    fprintf('******************************************\n');
    runtime = [runtime mpc.runtimeP2P+mpc.runtimeIteWS+mpc.runtimeOpt];
  end
  
  %------ Hybrid tracking controller ------
  % get the next virtual state to track
  [virtQuad,virtControl,EndFlag] = findNextStateODE(virtQuad,mpc,dt,simTime,TEB_list,iter,iterMax);  
  virtPos = [virtQuad(1);virtQuad(3)];
  virtStates(end+1,:)  = virtQuad';
  virtControls(end+1,:) = virtControl';
  
  % find relative state
  local_start = tic;
  rel_x = trueQuad.x - Q*virtQuad;
  
  % determine which controller to use, find optimal control, get spatial gradients
  XDims = 1:4;
  YDims = 5:8;
  
  deriv_TEB_ind = cell(4,1);
  for k = 1:4
    deriv_TEB_ind{k} = deriv{k}(:,:,:,:,TEB_ind);
  end
  pX = eval_u(sD.grid, deriv_TEB_ind, rel_x(XDims));
  pY = eval_u(sD.grid, deriv_TEB_ind, rel_x(YDims));

  % find optimal control of relative system (no performance control)
  uX = dynSys.optCtrl([], rel_x(XDims), pX, uMode);
  uY = dynSys.optCtrl([], rel_x(YDims), pY, uMode);

  u = [uX uY];
  trueControls = [trueControls; u];
  lookup_time = [lookup_time; toc(local_start)];
  
  %------ True System ------
  % add random disturbance to velocity within given bound
  d = dynSys.dMin + rand(2,1).*(dynSys.dMax - dynSys.dMin);
  
  % update state of true vehicle
  trueQuad.updateState(u, dt, [], d);
  truePos = [trueQuad.x(1); trueQuad.x(5)];
  trueStates = [trueStates; trueQuad.x'];
  
  %------ Find next TEB ------
  TEB_ind_x = get_TEB_ind(tau, sD, data, rel_x(1:4), TEB, min_level);
  TEB_ind_y = get_TEB_ind(tau, sD, data, rel_x(5:8), TEB, min_level);
 
  TEB_ind = min(TEB_ind_x, TEB_ind_y);
  TEB_list = flip(TEB(1:TEB_ind-1));
  TEB_cell{end+1} = TEB_list;
  
  % make sure error isn't too big (shouldn't happen)
  if norm(virtQuad - trueQuad.x([1 2 5 6])) > 10 % Modify this bound
    keyboard
  end
  
  % ------ Visualization ------
  if vis
    % plot local obstacles and position of the real system
    obsMap.plotLocal;
    obsMap.plotPadded;
    obsMap.plotSenseRegion(truePos,sense_range);    

    % plot current virtual state
    delete(hd)
    hd = plot(virtQuad(1),virtQuad(3),'g*','MarkerSize',5);
    
    % plot the entire trajectory of the virtual system
    plot(virtStates(:,1),virtStates(:,3),'color',[0.5 0.5 0.5],'LineStyle','-')
    
    % Plot the entire trajectory of the true system
    plot(trueStates(:,1),trueStates(:,5),'k')
    
    drawnow
  end
  
  simTime = simTime + dt;
  simTotalTime = simTotalTime + dt;
  simTimeSpan = [simTimeSpan; simTotalTime];
  
  TrackingError = [TrackingError;...
      norm((virtStates(end,1)-trueStates(end,1)),(virtStates(end,3)-trueStates(end,5)))];
  
  fprintf('Current time: %f \n',simTotalTime);
  
  iter = iter + 1;
end

obsMap.sense_update(truePos, sense_range, trackErr)
  
if vis
    delete(hV);
    hV = plotDisk([trueQuad.x(1);trueQuad.x(5)], PlantL, 'r-');  

    obsMap.plotLocal;
    obsMap.plotPadded;
    obsMap.plotSenseRegion(truePos,sense_range);
end

save virtStates virtStates
save virtControls virtControls
save trueStates trueStates
save trueControls trueControls
save MPC MPC

fprintf('\n******************************************\n');
fprintf('***         Navigation Complete        ***\n');
fprintf('******************************************\n');        
fprintf('***   Average MPC Runtime: %f s  ***\n', mean(runtime));
fprintf('***   Worst Case Runtime: %f s   ***\n', max(runtime));
fprintf('***     Total Sim Time: %f s    ***\n', simTotalTime);
fprintf('******************************************\n');

figure()
plot(simTimeSpan,TrackingError,'r.')
xlabel('Time (s)')
ylabel('Tracking Error')
legend('Optimal tracking')
grid on