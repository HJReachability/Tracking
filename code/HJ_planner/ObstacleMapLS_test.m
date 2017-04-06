function ObstacleMapLS_test()

%% Grid and Obstacles
L = 1;
g = createGrid([-L; -L], [L; L], [35; 35]);

obs1 = shapeEllipsoid(g, [0;0], [0.5;0.3]);
obs2 = shapeRectangleByCorners(g, [-0.75; -0.9], [-0.6; -0.7]);
obs3 = shapeSphere(g, [0.5; 0.1], 0.2);
obs = min(obs1, obs2);
obs = shapeDifference(obs, obs3);

%% Create sensing region
sense_region.g = createGrid([-0.6; -0.6], [0.6; 0.6], [101; 101]);
circle = shapeSphere(sense_region.g, [0;0], 0.5);
normal_vec = [1; 0];
lower_normal = rotate2D(normal_vec, -pi/6);
upper_normal = rotate2D(normal_vec, pi/6);

lower_boundary = shapeHyperplane(sense_region.g, lower_normal, [0;0]);
upper_boundary = shapeHyperplane(sense_region.g, upper_normal, [0;0]);

sense_region.data = shapeDifference(circle, lower_boundary);
sense_region.data = shapeDifference(sense_region.data, upper_boundary);

figure
visSetIm(sense_region.g, sense_region.data);
axis square

%% Test class
obsMap = ObstacleMapLS(g, obs);

figure
obsMap.plotGlobal;
hold on

while true
  [x, y] = ginput(2);
  p1 = [x(1); y(1)];
  p2 = [x(2); y(2)];
  p = [p1; atan2(p2(2)-p1(2), p2(1)-p1(1))];
  track_err = 0.1;
  
  obsMap.sense_update(p, sense_region, track_err);
  
  obsMap.plotLocal;
  obsMap.plotPadded;
  obsMap.plotSenseRegion;
  
end

end
