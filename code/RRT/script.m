obstacleFilename = 'obstacles.txt';
seedsPerAxis = 7;
treesMax = seedsPerAxis^3*3+2;
rrt = RrtPlanner(treesMax,seedsPerAxis,obstacleFilename);
rrt.SetStart([0 0.5 0]);
rrt.SetGoal([0 -0.5 0]);
rrt.drawingSkipsPerDrawing = 30;
plot3(rrt.smoothedPath(:,1),rrt.smoothedPath(:,2),rrt.smoothedPath(:,3),'k*');
rrt.Run()
delete(rrt);

% obstacleFilename = 'simpleobst.txt';
% seedsPerAxis = 7;
% treesMax = seedsPerAxis^3*3+2;
% rrt = RrtPlanner(treesMax,seedsPerAxis,obstacleFilename);
% rrt.drawingSkipsPerDrawing = 30;
% rrt = RrtPlanner(treesMax,seedsPerAxis,obstacleFilename);
% rrt.Run()
% plot3(rrt.path(:,1),rrt.path(:,2),rrt.path(:,3),'k*');