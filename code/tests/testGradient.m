function [deriv_answer] = testGradient(data, grid_min, grid_max, grid_N, x)
%`data` variable into, along with grid min and max, grid_N, etc. ...
...and query for gradient at a relative state?

%% inputs

%simple derivative function
derivFunc = @upwindFirstFirst;

grid_N = double(grid_N);

%% compute grid
g = createGrid(grid_min, grid_max, grid_N);

%% compute derivatives everywhere on grid
%[derivC, derivL, derivR] = computeGradients(g, data, derivFunc, upWind)
derivs = computeGradients(g,data,derivFunc);

%% Interpolate derivatives at point
deriv_answer = eval_u(g,derivs,x);
