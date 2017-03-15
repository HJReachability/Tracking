seedsPerAxis = 7;
treesMax = seedsPerAxis^3*3+2;

% arguments to change
obstacleFilename = 'obstacles.txt'; % file that has global obstacles, used for init setup
start = [0 -0.5 +0.2];
goal = [+0.0 +0.9 +0.4];
obsType = 'padded'; % obstacle type among padded, global, local. path is drawn according to this.
senseRange = 0.5;
trackErrBnd = 0.05;
plotGlobal = 1;
plotLocal = 1;
plotPadded = 1;
plotTrace = 1;
plotSmooth = 1;

% runs everything
rrt = RrtPlanner(treesMax, seedsPerAxis, obstacleFilename, obsType, senseRange, trackErrBnd,...
  start, goal, plotGlobal, plotLocal, plotPadded, plotTrace, plotSmooth); 

rrt.drawingSkipsPerDrawing = 5;
rrt.Run()
%plot3(rrt.path(:,1),rrt.path(:,2),rrt.path(:,3),'k*');

path = unique(double(rrt.smoothedPath), 'rows'); % output smoothed path
nextPoint = path(2, :); % point that rrt would head towards
direction = (nextPoint - start)./norm(nextPoint - start); % unit vector of direction to go towards
% newState = start + delta_x.*direction;

% can call rrt.obsmap.obstaclePlot(1, 1, 0)