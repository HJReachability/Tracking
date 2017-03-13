obstacleFilename = 'simpleobst.txt'; % with track bound, should only have 18 obstacles...
seedsPerAxis = 7;
treesMax = seedsPerAxis^3*3+2;
rrt = RrtPlanner(treesMax,seedsPerAxis,obstacleFilename);
rrt.drawingSkipsPerDrawing = 5;
rrt.SetStart([0 -0.9 0]);
rrt.SetGoal([0 0.9 0]);
rrt.Run()
plot3(rrt.path(:,1),rrt.path(:,2),rrt.path(:,3),'k*');