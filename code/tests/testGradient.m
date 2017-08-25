function [error, deriv_answer] = testGradient(x, derivGuess, derivFunc)
%% inputs
% state to test at
if nargin <1
x = [.1, .1, .1];
end

% gradients to compare to
if nargin <2
derivGuess = [-.2736; .4917; .0200];
end

% function for computing derivatives
if nargin <3
  derivFunc = @upwindFirstFirst; 
  %can also try @upwindFirstWENO5 for more fidelity
end

% how close they can be
small = 0.001*ones(3,1);

% load grid and data info
load('test_value_function.mat')
grid_N = double(grid_N);

%% compute grid
g = createGrid(grid_min, grid_max, 10);

%% compute derivatives everywhere on grid
%[derivC, derivL, derivR] = computeGradients(g, data, derivFunc, upWind)
Derivs = computeGradients(g,data,derivFunc); 

%% Interpolate derivatives at point
deriv_answer = eval_u(g,Derivs,x);

error = abs(deriv_answer - derivGuess);
if error <= small
  disp('test passed!')
else
  disp('error is ')
  disp(num2str(error))
  disp('deriv_answer was')
  disp(num2str(deriv_answer))
end
