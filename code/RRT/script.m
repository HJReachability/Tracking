obstacleFilename = 'obstacles.txt';
seedsPerAxis = 7;
treesMax = seedsPerAxis^3*3+2;
rrt = RrtPlanner(treesMax,seedsPerAxis,obstacleFilename);
rrt.obs

% rrt.drawingSkipsPerDrawing = 30;
% rrt = RrtPlanner(treesMax,seedsPerAxis,obstacleFilename);
% rrt.Run()
% plot3(rrt.path(:,1),rrt.path(:,2),rrt.path(:,3),'k*');

% obsmap = ObstacleMap(rrt);
% obsmap.global_obs
