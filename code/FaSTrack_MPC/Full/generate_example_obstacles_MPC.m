function generate_example_obstacles_MPC()

s = 1.0;

% Obstacle 1
obs1 = {[0.75 2.0]*s [3.25 2.0]*s [3.25 4.5]*s [0.75 4.5]*s};

% Obstavle 2
obs2 = {[8.0 2.0]*s [10.5 2.0]*s [10.5 4.5]*s [8.0 4.5]*s};

% Obstavle 3
obs3 = {[4.0 8.5]*s [8.0 8.5]*s [8.0 9.0]*s [4.0 9.0]*s};

vOb = [obs1; obs2; obs3];

save vOb vOb

end