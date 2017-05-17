function [newStates, p, rrt] = rrtNextState(start, goal, obs, ...
  delta_x, rrtSoFar, vis)
% [newState, path, rrt] = rrtNextState(delta_x, start) 

if nargin < 1
  start = [0.00 -0.5 0.2];
end

if iscolumn(start)
  start = start';
end

if nargin < 2
  goal = [0.00 0.9 0.4];
end

if iscolumn(goal)
  goal = goal';
end


if nargin < 3
  load('obs.mat');
end

if nargin < 4
  delta_x = 0.1;
end

if nargin < 5
  rrtSoFar = [];
end

if nargin < 6
  vis = true;
end

seedsPerAxis = 7;
treesMax = seedsPerAxis^3*3+2;

% runs everything
rrt = RrtPlanner(treesMax, seedsPerAxis, obs, rrtSoFar, start, goal);

rrt.drawingSkipsPerDrawing = 5;
rrt.Run()

% Remove points close together in smoothed path, and keep same sorted order
p = rrt.smoothedPath;
[~, ia] = uniquetol(p, 1e-3, 'ByRows', true);
p = p(sort(ia), :);

if isempty(p)
  newStates = [];
  rrt = [];
  return
else
  p(1,:) = [];
end
prev_point = start;
newStates = start;
while ~isempty(p)
  nextPoint = p(1, :); % point that rrt would head towards
  
  dist = norm(nextPoint - prev_point);
  steps = ceil(dist/delta_x);
  direction = (nextPoint - prev_point)/dist;
  
  for i = 1:steps
    newStates = [newStates; newStates(end,:) + delta_x*direction];
  end
  
  prev_point = p(1,:);
  p(1,:) = [];
end

newStates(1,:) = [];

if vis
  figure
  plot3(p(:,1), p(:,2), p(:,3), '.-')
  hold on
  plot3(start(1), start(2), start(3), 'o')
  plot3(goal(1), goal(2), goal(3), '*')
  
  obsMap = ObstacleMap(obs);
  obsMap.plotGlobal;
end

end