function generate_example_obstacles_FSM()

L = 1;
g = createGrid([-L; -L; 0], [L; L; 2*pi], [35; 35; 35], 3);
g2D = proj(g, [], [0 0 1]);

% L shape
obs1 = shapeRectangleByCorners(g2D, [-0.5; -0.1], [0; 0]);
obs2 = shapeRectangleByCorners(g2D, [-0.1; -0.5], [0; 0]);

% Right obstacle
obs3 = shapeRectangleByCorners(g2D, [0.2; -0.1], [0.5; 0]);
obs5 = shapeSphere(g2D, [0.5; -0.1], 0.1);

% Left obsdtacle
obs4 = shapeRectangleByCorners(g2D, [-0.1; 0.2], [0; 0.5]);
obs6 = shapeSphere(g2D, [-0.1; 0.5], 0.1);


obs2D = min(obs1, obs2);
obs2D = min(obs2D, obs3);
obs2D = min(obs2D, obs4);
obs2D = min(obs2D, obs5);
obs2D = min(obs2D, obs6);

figure
visSetIm(g2D, obs2D);

obs = repmat(obs2D, [1 1 g.N(3)]);
axis square
save('obs.mat', 'L', 'g2D', 'obs2D', 'g', 'obs', '-v7.3')
end