obstacleFilename = 'simpleobst.txt';
seedsPerAxis = 7;
treesMax = seedsPerAxis^3*3+2;
rrt = RrtPlanner(treesMax,seedsPerAxis,obstacleFilename);
rrt.drawingSkipsPerDrawing = 5;
rrt.SetStart([-0.5 0 0.2]);
rrt.SetGoal([0.9 0 0.2]);
rrt.Run()
plot3(rrt.path(:,1),rrt.path(:,2),rrt.path(:,3),'k*');