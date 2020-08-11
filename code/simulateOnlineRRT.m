function [trueQuad, virt_x, controller] = ...
  simulateOnlineRRT(sD_X, derivX, sD_Z, derivZ, TEB, obs, extraArgs)
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

%addpath(genpath('.'))

% % Video
% vout = VideoWriter('figsRRT1/videoRRT.mp4', 'MPEG-4');
% vout.Quality = 100;
% vout.FrameRate = 10;
% vout.open;

% Problem setup
start = [-12; 0; 0];
goal = [12; 0; 0];
small = 0.1;
virt_v = sD_X.dynSys.uMax(1);

dt = 0.001;
delta_x = virt_v*dt;

% Subsystems
XDims = 1:4;
YDims = 5:8;
ZDims = 9:10;

if nargin < 7
  extraArgs = [];
end


% matrix to compare position states (virt vs. true)
if ~isfield(extraArgs,'Q')
  Q = zeros(10,3);
  Q(1,1) = 1;
  Q(5,2) = 1;
  Q(9,3) = 1;
end

if isfield(extraArgs, 'visualize')
  vis = extraArgs.visualize;
else
  vis = true;
end

%% Before Looping
trackErr = TEB;
senseRange = 2*TEB + small;


uMode = 'min';
% dMode = 'max'; % Not needed since we're not using worst-case control

obsMap = ObstacleMapRRT(obs);

% plot global obstacles
if vis
  f = figure(1);
  clf
  f.Color = 'white';
  %f.Position = [100 100 1280 720];
  
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
start_x = zeros(10,1);
start_x([1 5 9]) = start;


% Create real quadrotor system
rl_ui = [2 4 6];
trueQuad = Quad10D(start_x, sD_X.dynSys.uMin(rl_ui), sD_X.dynSys.uMax(rl_ui), ...
  sD_X.dynSys.dMin, sD_X.dynSys.dMax, 1:10);

% % define when to switch from safety control to performance control
% small = 1;
% safetyBound = bubble - small.*[gs{1}.dx(1); gs(2).dx(1); gs(3).dx(1)];

%% Start loop! Tracking error bound block
%input: environment, sensing
%output: augmented obstacles
newStates = [];
iter = 1;
global_start = tic; % Time entire simulation

max_iter = 50000;
lookup_time = 0;
plan_time = 0;

virt_x = nan(3, max_iter);
virt_x(:,1) = start;

controller = nan(1, max_iter-1);
% while we haven't reached the final iteraction and we haven't reached the
% goal
while iter < max_iter && norm(trueQuad.x([1 5 9]) - goal) > 0.5
  iter = iter + 1;

  % 1. Sense your environment, locate obstacles
  % 2. Expand sensed obstacles by tracking error bound
  sensed_new = obsMap.sense_update(trueQuad.x([1 5 9]), senseRange, trackErr);
  
  %% Path Planner Block
  % Replan if a new obstacle is seen
  plan_start = tic;
  if isempty(newStates) || sensed_new
    % Update next virtual state
    newStates = rrtNextState(virt_x(:,iter-1), goal, obsMap.padded_obs, ...
        delta_x, [], false);
    %(trueQuad.x([1 5 9]), goal, obsMap.padded_obs, ...
    %  delta_x, [], false);
  end
  plan_time = plan_time + toc(plan_start);
  virt_x(:,iter) = newStates(1,:)';
  newStates(1,:) = [];

  %% Hybrid Tracking Controller
  % 1. find relative state
  look_up_start = tic;
  rel_x = trueQuad.x - Q*virt_x(:,iter);
  
  % 2. Determine which controller to use, find optimal control
  %get spatial gradients
  pX = eval_u(sD_X.grid, derivX, rel_x(XDims));
  pY = eval_u(sD_X.grid, derivX, rel_x(YDims));
  pZ = eval_u(sD_Z.grid, derivZ, rel_x(ZDims));
  
  if norm(virt_x(:,iter) - trueQuad.x([1 5 9])) > 0.5*trackErr
      uX = sD_X.dynSys.optCtrl([], rel_x(XDims), pX, uMode);
      uY = sD_X.dynSys.optCtrl([], rel_x(YDims), pY, uMode);
      uZ = sD_Z.dynSys.optCtrl([], rel_x(ZDims), pZ, uMode);
      u = [uX uY uZ];
      u = u(rl_ui);
      up = u;
      boxColor = 'r';
      
      controller(iter-1) = 1; % reachability controller
      control_type = 'optimal';
      
   else
       uX = LQR_Q2D(rel_x(XDims),.5);
       uY = LQR_Q2D(rel_x(YDims),.5);
       uZ = LQR_Q2D(rel_x(ZDims),.01);
       
       u = [uX uY uZ];
       boxColor = 'b';
       
       controller(iter-1) = 0; % LQR controller
       control_type = 'lqr';
  end
   
  % Find optimal control of relative system (no performance control)
  lookup_time = lookup_time + toc(look_up_start);
  
  %% True System Block
  % 1. add random disturbance to velocity within given bound
  d = sD_X.dynSys.dMin + rand(3,1).*(sD_X.dynSys.dMax - sD_X.dynSys.dMin);
  
  % 2. update state of true vehicle
  trueQuad.updateState(u, dt, [], d);

  % Make sure error isn't too big (shouldn't happen)
  max(abs(virt_x(:,iter) - trueQuad.x([1 5 9])))
%   if max(virt_x - trueQuad.x([1 5 9])) > 1.5*trackErr
%     keyboard
%   end
  
  %% Virtual System Block  
%   fprintf('Iteration took %.2f seconds\n', toc);
  
  % Visualize
  if (vis && ~mod(iter,25)) || iter == 2
    % Local obstacles and true position
    obsMap.plotLocal;
    if strcmp(control_type,'optimal')
    plot3(trueQuad.x(1), trueQuad.x(5), trueQuad.x(9), 'r.')
    else
    plot3(trueQuad.x(1), trueQuad.x(5), trueQuad.x(9), 'b.')
    end
    hold on
    
    % Virtual state
    if exist('hV', 'var')
      hV.XData = virt_x(1,iter);
      hV.YData = virt_x(2,iter);
      hV.ZData = virt_x(3,iter);
    else
      hV = plot3(virt_x(1,iter), virt_x(2,iter), virt_x(3,iter), '*', ...
        'color', [0 0.75 0]);
    end
    
    % Tracking error bound
    left_bd = virt_x(1,iter) - trackErr;
    right_bd = virt_x(1,iter) + trackErr;
    back_bd = virt_x(2,iter) - trackErr;
    front_bd = virt_x(2,iter) + trackErr;
    bottom_bd = virt_x(3,iter) - trackErr;
    top_bd = virt_x(3,iter) + trackErr;
    
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
    
    if iter == 2
      boxMap = ObstacleMapRRT(boxShape);
    else
      boxMap.global_obs = boxShape;
    end
    boxMap.plotGlobal(boxColor, '-');

    title(sprintf('t = %.1f', dt*(iter-1)));
    drawnow

%     export_fig(sprintf('figsRRT2/%d', iter), '-png')
%     savefig(sprintf('figsRRT2/%d.fig', iter))
%     
%     current_frame = getframe(gcf); % gca does just the plot
%     writeVideo(vout, current_frame);    
  end
end

virt_x(:,iter+1:end) = [];
controller(iter+1:end) = [];
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
