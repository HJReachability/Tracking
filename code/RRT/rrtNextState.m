function [newState, p, rrt] = rrtNextState(start, goal, obs, ...
  delta_x, rrtSoFar, vis)
% [newState, path, rrt] = rrtNextState(delta_x, start) 

if nargin < 1
  start = [0.00 -0.5 0.2];
end

if nargin < 2
  goal = [0.00 0.9 0.4];
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
[~, ia] = uniquetol(p, 1e-4, 'ByRows', true);
p = p(sort(ia), :);

nextPoint = p(2, :); % point that rrt would head towards
direction = (nextPoint - start)./norm(nextPoint - start); % unit vector of direction to go towards
newState = start + delta_x.*direction;

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