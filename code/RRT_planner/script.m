seedsPerAxis = 7;
treesMax = seedsPerAxis^3*3+2;

% arguments to change

start = [0 -0.5 +0.2];
goal = [+0.0 +0.9 +0.4];
obsType = 'padded'; % obstacle type among padded, global, local. path is drawn according to this.
senseRange = 0.5;
trackErrBnd = 0.05;
plotGlobal = 1;
plotLocal = 0;
plotPadded = 1;
plotTrace = 1;
plotSmooth = 1;

% runs everything
rrtSoFar = [];
rrt = RrtPlanner(treesMax, seedsPerAxis, obstacleFilename, rrtSoFar, obsType, senseRange, trackErrBnd,...
start, goal, plotGlobal, plotLocal, plotPadded, plotTrace, plotSmooth); 

rrt.drawingSkipsPerDrawing = 5;
rrt.Run()
%plot3(rrt.path(:,1),rrt.path(:,2),rrt.path(:,3),'k*');

path = unique(double(rrt.smoothedPath), 'rows'); % output smoothed path
nextPoint = path(2, :); % point that rrt would head towards
direction = (nextPoint - start)./norm(nextPoint - start); % unit vector of direction to go towards

% MINE:
% tests whether it keeps old rrt // computation time should be a lot faster
newState = start + 0.02.*direction;
rrtSoFar = rrt.rrt;
rrt = RrtPlanner(treesMax, seedsPerAxis, obstacleFilename, rrtSoFar, obsType, senseRange, trackErrBnd,...
newState, goal, plotGlobal, plotLocal, plotPadded, plotTrace, plotSmooth); 

rrt.drawingSkipsPerDrawing = 5;
rrt.Run()

% can call rrt.obsmap.obstaclePlot(1, 1, 0)