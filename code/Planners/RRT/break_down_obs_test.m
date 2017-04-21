function break_down_obs_test()

const_dim = randi(3);
const_dim_logical = false(3, 1);
const_dim_logical(const_dim) = true;

in = zeros(4, 3);
lower = -20*rand(2,1);
upper = 20*rand(2,1);

in(:, ~const_dim_logical) = [lower(1) upper(1); lower(1) upper(2); ...
  lower(2) upper(2); lower(2) upper(1)];


max_size = 2 + 3 * rand;
out = break_down_obs(in, max_size);
obsMap_in = ObstacleMap(in);
obsMap_out = ObstacleMap(out);

figure;
subplot(1,2,1)
obsMap_in.plotGlobal;

subplot(1,2,2)
obsMap_out.plotGlobal;
end