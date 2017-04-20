function sense_region = create_sensing_region(range, angle)
% sense_region = create_sensing_region(range, angle)
%     Creates a sensing region in 2D in the shape of a portion of a circle with 
%     radius range, with angle in [-angle, angle]

sense_region.g = createGrid(-1.1*range*ones(2,1), 1.1*range*ones(2,1), 101);
circle = shapeSphere(sense_region.g, [0;0], range);

lower_normal = rotate2D([1; 0], -angle);
upper_normal = rotate2D([1; 0], angle);

lower_boundary = shapeHyperplane(sense_region.g, lower_normal, [0;0]);
upper_boundary = shapeHyperplane(sense_region.g, upper_normal, [0;0]);

sense_region.data = shapeDifference(circle, lower_boundary);
sense_region.data = shapeDifference(sense_region.data, upper_boundary);
end