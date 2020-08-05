function simulateOnlineRRTSwitching(Mode, obs_filename, extraArgs)
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
%           .vis          -  visualize
%           .movie        - make a movie
%       inputs related to sensing environment
%       inputs related to RRT (goal?)

addpath(genpath('.'))

% Problem setup
start = [-12; 0; 0];
goal = [15; 0; 0];

% Subsystems
XDims = 1:4;
YDims = 5:8;
ZDims = 9:10;

% 
% % Mode 1 (fast)
% if nargin <1
%   data_filenames{1} = 'Q10D_Q3D_uplan_15_tenth.mat';
% %  data_filenames{2} = 'Q10D_Q3D_Rel_u_10_tenth.mat';
%   data_filenames{2} = 'Q10D_Q3D_uplan_5_tenth.mat';
% %  data_filenames{4} = 'Q10D_Q3D_Rel_u_1_tenth.mat';
% end

% obstacles
if nargin <2
  obs_filename = 'obs.mat';
end
load(obs_filename)

%extraArgs
if nargin <3
  extraArgs.vis = 1;
  extraArgs.auto = 1;
  extraArgs.human = 0;
end

if isfield(extraArgs,'movie')&&extraArgs.movie
  gif_out_filename = './visualize_modes_RRT_switching.gif';  %Setup filename for gif
  avi_out_filename = './visualize_modes_RRT_switching.avi'; 
  v = VideoWriter(avi_out_filename);
  v.FrameRate = 10;   %Set framerate of playback
  open(v)   %Opens the file for writing. Make sure to close at the end!!
end

% matrix to compare position states (virt vs. true)
if ~isfield(extraArgs,'Q')
  Q = zeros(10,3);
  Q(1,1) = 1;
  Q(5,2) = 1;
  Q(9,3) = 1;
end

uMode = 'min';



%modeNum = length(data_filenames);
modeNum = length(Mode);
%% Before Looping
dt = 0.1;
senseRange = 6;
tGreedy = 5; %switching look-ahead time

% Mode = cell(1,modeNum);
obsMap = cell(1,modeNum);

%get all the info about the different modes
for i = 1:modeNum
%   Mode{i} = load(data_filenames{i});
%   Mode{i}.derivX = computeGradients(Mode{i}.sD_X.grid,Mode{i}.dataX);
%   Mode{i}.derivZ = computeGradients(Mode{i}.sD_Z.grid,Mode{i}.dataZ);
%   Mode{i}.trackErr = Mode{i}.TEB + .1;
%   Mode{i}.virt_v = Mode{i}.sD_X.dynSys.uMax(1);
%   Mode{i}.delta_x = Mode{i}.virt_v*dt;
  obsMap{i} = ObstacleMapRRT(obs);
end

% plot global obstacles
if isfield(extraArgs,'vis')&&extraArgs.vis
  az = -20;
  el = 20;
  f = figure(1);
  clf
  f.Color = 'white';
  f.Position = [100 100 1280 720];
  
  obsMap{1}.plotGlobal()
  f.Children(end).FontSize = 16;
  hold on
  plot3(goal(1), goal(2), goal(3), 'bo')
  plot3(start(1), start(2), start(3), 'ro')
  
  xlabel('x', 'FontSize', 16)
  ylabel('y', 'FontSize', 16)
  zlabel('z', 'FontSize', 16)
  
  axis equal
  xlim([-15 20])
  %
  %   ylim([-10 10])
  %   zlim([-10 10])
  view([az el])
  box on
  grid on
end

% set initial states to zero
start_x = zeros(10,1);
start_x([1 5 9]) = start;

% Create real quadrotor system
rl_ui = [2 4 6];
for i = 1:modeNum-1
  if Mode{i}.sD_X.dynSys.uMin(rl_ui) ~= Mode{i+1}.sD_X.dynSys.uMin(rl_ui)
    error('real system mismatch!')
  end
end

trueQuad = Quad10D(start_x, Mode{1}.sD_X.dynSys.uMin(rl_ui), ...
  Mode{1}.sD_X.dynSys.uMax(rl_ui), Mode{1}.sD_X.dynSys.dMin, ...
  Mode{1}.sD_X.dynSys.dMax, 1:10);

% % define when to switch from safety control to performance control
% small = 1;
% safetyBound = bubble - small.*[gs{1}.dx(1); gs(2).dx(1); gs(3).dx(1)];

%% Start loop! Tracking error bound block
%input: environment, sensing
%output: augmented obstacles
newStates = [];
virt_x = start;
iter = 0;
global_start = tic; % Time entire simulation

max_iter = 5000;
lookup_time = 0;

%initialize stuff
m = 1;
sensed_new = cell(1,modeNum);
[sensed_new{:}] = deal(0);
counter = 0;

while iter < max_iter && norm(trueQuad.x([1 5 9]) - goal) > 0.5
  iter = iter + 1;
  counter = counter+1;
  
  % 1. Sense your environment, locate obstacles
  % 2. Expand sensed obstacles by tracking error bound
  for j = 1:modeNum
    sensed_new{j} = obsMap{j}.sense_update(trueQuad.x([1 5 9]),...
      senseRange, Mode{j}.trackErr);
  end
  
  %% Path Planner Block
  % Replan if a new obstacle is seen
  
  if isempty(newStates) || sensed_new{m} || counter > 50
    
    % Update next virtual state
    mlast = m;
    
    % human switching
    if isfield(extraArgs,'human')&&extraArgs.human %if human switching
      
      [m, newStates_test] = humanSwitch(mlast,virt_x,goal,trueQuad,...
        Mode,obsMap,modeNum);

    % auto switching  
    elseif isfield(extraArgs,'auto')&&extraArgs.auto %if auto switching
      
      [m, newStates_test] = autoSwitch(mlast,virt_x,goal,trueQuad,...
         Mode,obsMap,modeNum,tGreedy,dt);

    else
      error('auto or human?')
    end
    
    if counter <= 300 && ... %if we haven't reached 300 counts
        ~sensed_new{m} && ... %and we didn't sense anything new
        ~isempty(newStates) &&... %and we aren't out of places to go
        mlast == m && ... %and we're staying in the same mode
        norm(trueQuad.x([1 5 9]) - goal) > .5 %and we're not near the goal
      
      %then stick with the current plan
      newStates_test = newStates;
      
    end
    
    counter = 0; %reset counter
    newStates= newStates_test; %set next states
    
  end
  
  if isempty(newStates)
    keyboard
  end
    
  title(['Mode ' num2str(m)])
  virt_x = newStates(1,:)';
  newStates(1,:) = [];
  
  %% Hybrid Tracking Controller
  % 1. find relative state
  local_start = tic;
  rel_x = trueQuad.x - Q*virt_x;
  
  % 2. Determine which controller to use, find optimal control
  %get spatial gradients
  pX = eval_u(Mode{m}.sD_X.grid, Mode{m}.derivX, rel_x(XDims));
  pY = eval_u(Mode{m}.sD_X.grid, Mode{m}.derivX, rel_x(YDims));
  pZ = eval_u(Mode{m}.sD_Z.grid, Mode{m}.derivZ, rel_x(ZDims));
  
  % Find optimal control of relative system (no performance control)
  uX = Mode{m}.sD_X.dynSys.optCtrl([], rel_x(XDims), pX, uMode);
  uY = Mode{m}.sD_X.dynSys.optCtrl([], rel_x(YDims), pY, uMode);
  uZ = Mode{m}.sD_Z.dynSys.optCtrl([], rel_x(ZDims), pZ, uMode);
  u = [uX uY uZ];
  u = u(rl_ui);
  lookup_time = lookup_time + toc(local_start);
  
  %% Tracking System Block
  % 1. add random disturbance to velocity within given bound
  d = Mode{m}.sD_X.dynSys.dMin + rand(3,1).*(Mode{m}.sD_X.dynSys.dMax - Mode{m}.sD_X.dynSys.dMin);
  
  % 2. update state of true vehicle
  trueQuad.updateState(u, dt, [], d);
  
  % Make sure error isn't too big (shouldn't happen)
  if norm(virt_x - trueQuad.x([1 5 9])) > (Mode{m}.TEB + .1)
    keyboard
  end
  
  %% Virtual System Block
  %   fprintf('Iteration took %.2f seconds\n', toc);
  
  % Visualize
  if isfield(extraArgs,'vis')&&extraArgs.vis
    colors = {'b.','g.', 'c.','m.','y.','k.'};
    
    % Local obstacles and true position
    obsMap{m}.plotLocal;
    plot3(trueQuad.x(1), trueQuad.x(5), trueQuad.x(9), colors{m})
    hold on
    
    % Virtual state
    if exist('hV', 'var')
      hV.XData = virt_x(1);
      hV.YData = virt_x(2);
      hV.ZData = virt_x(3);
    else
      hV = plot3(virt_x(1,end), virt_x(2,end), virt_x(3,end), '*', 'color', [0 0.75 0]);
    end
    
    if iter > 20 && iter < 95
      az = az-1;
      el = el+.8;
      view(az,el)
    elseif iter >= 95 && iter <= 170
      az = az-1;
      el = el-.8;
      view(az,el)
    end
    
    % Tracking error bound
    left_bd = virt_x(1) - Mode{m}.trackErr;
    right_bd = virt_x(1) + Mode{m}.trackErr;
    back_bd = virt_x(2) - Mode{m}.trackErr;
    front_bd = virt_x(2) + Mode{m}.trackErr;
    bottom_bd = virt_x(3) - Mode{m}.trackErr;
    top_bd = virt_x(3) + Mode{m}.trackErr;
    
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
    if isfield(extraArgs,'movie')&&extraArgs.movie
      frame = getframe(gcf);
      image_data = frame2im(frame);
      [imind,cm] = rgb2ind(image_data,256);
      if iter == 1
        imwrite(imind,cm,gif_out_filename,'gif', 'Loopcount',inf);
      else
        imwrite(imind,cm,gif_out_filename,'gif','WriteMode','append');
      end
      writeVideo(v,image_data)
    end

%     export_fig(sprintf('pics/%d', iter), '-png')
%     savefig(sprintf('pics/%d.fig', iter))    
  end
end

comp_time = toc(global_start);
fprintf('%d iterations in %.4f seconds\n', iter, comp_time)
fprintf('%.4f seconds per iteration on average\n', comp_time/iter)

fprintf('%d iterations in %.4f seconds\n', iter, lookup_time)
fprintf('%.4f seconds per iteration on average\n', lookup_time/iter)

if isfield(extraArgs,'movie')&&extraArgs.movie
  k = 0;
  while k <= 150
    if k > 0 && k < 75
      %virt_x(1)> -8 && az > -95
      az = az+1;
      el = el+.8;
      view(az,el)
    elseif k >= 75 && k <= 150%virt_x(1) > -8 && az <= -95 && az >= -170
      az = az+1;
      el = el-.8;
      view(az,el)
    end
    k = k+1;
     frame = getframe(gcf);
      image_data = frame2im(frame);
      [imind,cm] = rgb2ind(image_data,256);
      imwrite(imind,cm,gif_out_filename,'gif','WriteMode','append');
      writeVideo(v,image_data)
  end
     close(v);
end
end

function [m,newStates_test]= humanSwitch(mlast,virt_x,goal,trueQuad,...
        Mode,obsMap,modeNum)
      
      %pick which mode
      m =input('which mode? ');
      if ~(1<=m<=modeNum)
        m = input('which mode? ');
      end
      
      %update virtual state
      newStates_test = rrtNextState((virt_x), goal, ...
        obsMap{m}.padded_obs, Mode{m}.delta_x, [], false);
      
      if ~isempty(newStates_test) && ...
          (norm(newStates_test(1,:)'-trueQuad.x([1 5 9])) > Mode{m}.trackErr)
        % if path but no feasible path
        
        m = mlast; %go back to previous mode
        
        newStates_test = rrtNextState((virt_x), goal, ...
          obsMap{m}.padded_obs, Mode{m}.delta_x, [], false);
      
      elseif isempty(newStates_test) % if no path
        
        %move up a mode
        while isempty(newStates_test) ||...
          (norm(newStates_test(1,:)'-trueQuad.x([1 5 9])) > Mode{m}.trackErr)
        % while no path or no feasible path
        
          m = m+1;
          if m > modeNum %if we've run out of modes
            error('no feasible path found!')
          end
          
          newStates_test = rrtNextState(virt_x, goal, ...
            obsMap{m}.padded_obs, Mode{m}.delta_x, [], false);
        end
        %stop when we have a non-empty feasible virtual state
      end
end


function [m, newStates] = autoSwitch(mlast,virt_x,goal,trueQuad,...
    Mode,obsMap,modeNum,tGreedy,dt)

%index previous, current, and next nodes

mvec = [];
mlegend = {};

% previous
if mlast(1) > 1
    mvec(end+1) = mlast -1;
    mlegend{length(mvec)} = 'previous';
end

% current
mvec(end+1) = max(1,mlast-1);
mlegend{length(mvec)} = 'current';

% next
if mlast(1) < modeNum
    mvec(end+1) = mvec(1) +1;
    mlegend{length(mvec)} = 'next';
end

%test for this mode and the next
empty_flag = zeros(size(mvec));
parfor ii = 1:length(mvec)
    newStates_test{ii} = rrtNextState((virt_x), goal, ...
        obsMap{mvec(ii)}.padded_obs, Mode{mvec(ii)}.delta_x, [], false);
    
    %       newStates_test2 = rrtNextState((virt_x),goal, ...
    %         obsMap{m+1}.padded_obs,Mode{m+1}.delta_x, [], false);
    if isempty(newStates_test{ii})
        empty_flag(ii) = 1;
    end
end



if sum(empty_flag) == 0
    % if the modes aren't empty, pick the mode that is closest to
    % the goal in the next 5 seconds
    
    timeLeft = Inf;
    parfor ii = 1:length(empty_flag)
            timeLeft = min(timeLeft,length(newStates_test{ii}));

    end
    
    timeCheck = min(tGreedy/dt,timeLeft);
    
    parfor ii = 1:length(mvec)
        goaldist(ii) = norm(newStates_test{ii}(timeCheck,:)'-goal,Inf);
    end
    
    [~,idx] = min(goaldist);
    
    %val = goaldist(2)-bestdist; %which mode thinks it'll get you closest to the goal?
    
%     if val > 0.05 && ... % if this value is high enough
%             (norm(newStates_test{idx}(1,:)'-trueQuad.x([1 5 9])) < Mode{idx}.trackErr)
   if (norm(newStates_test{idx}(1,:)'-trueQuad.x([1 5 9])) < Mode{idx}.trackErr)

        %and switching won't violate new TEB
        
        %then go to that node
        m = mvec(idx);
        newStates = newStates_test{idx};
   else
        idx = find(contains(mlegend,'current'));
         m = mvec(idx);
         newStates = newStates_test{idx};
    end
elseif sum(empty_flag) > 0 && sum(empty_flag) < length(empty_flag)

    timeLeft = Inf;
    for ii = 1:length(empty_flag)
        if empty_flag(ii) == 0
            timeLeft = min(timeLeft,length(newStates_test{ii}));
        end
    end
    
    timeCheck = min(tGreedy/dt,timeLeft);
    
    goaldist = inf*ones(length(mvec));
    parfor ii = 1:length(mvec)
        if empty_flag(ii) == 0
            goaldist(ii) = norm(newStates_test{ii}(timeCheck,:)'-goal,Inf);
        end
    end
    
        
    [~,idx] = min(goaldist);
    
    %val = goaldist(2)-bestdist; %which mode thinks it'll get you closest to the goal?
    
%     if val > 0.05 && ... % if this value is high enough
%             (norm(newStates_test{idx}(1,:)'-trueQuad.x([1 5 9])) < Mode{idx}.trackErr)
   if (norm(newStates_test{idx}(1,:)'-trueQuad.x([1 5 9])) < Mode{idx}.trackErr)

        %and switching won't violate new TEB
        
        %then go to that node
        m = mvec(idx);
        newStates = newStates_test{idx};
   else
        idx = find(contains(mlegend,'current'));
        
        if empty_flag(idx) == 0
         m = mvec(idx);
         newStates = newStates_test{idx};
        else
            error('no safe path found!')
        end
    end
else
    error('no path found!')
end
    
% elseif isempty(newStates_test{1}) ...
%         && ~isempty(newStates_test{2})...
%         
%     if ~isempty(newStates_test{3})
%         timeLeft = min(length(newStates_test{2}),length(newStates_test{3}));
%         
%         timeCheck = min(tGreedy/dt,timeLeft);
%         
%         goaldist(1) = Inf;
%         parfor ii = 2:length(mvec)
%             goaldist(ii) = norm(newStates_test{ii}(timeCheck,:)'-goal,Inf);
%         end
%          
%         val = goaldist(3)-goaldist(2); %which mode thinks it'll get you closest to the goal?
%         
%         if val > 0.05 && ... % if this value is high enough
%                 (norm(newStates_test{3}(1,:)'-trueQuad.x([1 5 9])) < Mode{3}.trackErr)
%             %and switching won't violate new TEB
%             
%             %then go to that node
%             m = mvec(3);
%             newStates = newStates_test{3};
%         else
%             m = mvec(2);
%             newStates = newStates_test{2};
%         end
%     
%     else
%         % set 2 to default
%         m = mvec(2);
%         newStates = newStates_test{2};
%     end
%     
%     
%     
%     
% %     if (norm(newStates_test{2}(1,:)'-trueQuad.x([1 5 9])) < Mode{2}.trackErr)
% %         m = mvec(2);
% %         newStates = newStates_test{2};
% %         
% %     elseif (norm(newStates_test{3}(1,:)'-trueQuad.x([1 5 9])) < Mode{3}.trackErr)
% %         m = mvec(3);
% %         newStates = newStates_test{3};
% %     else
% %         error('no path feasible!')
% %     end
%     
% elseif isempty(newStates_test{1}) ...
%         && isempty(newStates_test{2})...
%         && ~isempty(newStates_test{3})
%     
%     if (norm(newStates_test{3}(1,:)'-trueQuad.x([1 5 9])) < Mode{3}.trackErr)
%         m = mvec(3);
%         newStates = newStates_test{3};
%     else
%         error('no path feasible!')
%     end
% else
%     error('no path found!')
% end
end