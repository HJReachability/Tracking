function generate_example_obstacles()

% Plane 1
x1 = -9;
obs1_large = [x1 -10 -10; x1 -10 5; x1 5 5; x1 5 -10];

x2 = -3;
obs2_large = [x2 -5 -10; x2 -5 5; x2 10 5; x2 10 -10];

x3 = 3;
obs3_large = [x3 -5 -5; x3 -5 10; x3 10 10; x3 10 -5];

x4 = 9;
obs4_large = [x4 -10 -5; x4 -10 10; x4 5 10; x4 5 -5];

obs_large = cat(3, obs1_large, obs2_large);
obs_large = cat(3, obs_large, obs3_large);
obs_large = cat(3, obs_large, obs4_large);

obsMap_large_piece = ObstacleMap(obs_large);

figure
obsMap_large_piece.plotGlobal

max_size_out = 3;
obs = break_down_obs(obs_large, max_size_out);
obsMap = ObstacleMap(obs);

figure
obsMap.plotGlobal;

save('obs.mat', 'obs', '-v7.3')

end