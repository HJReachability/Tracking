function P5D_Dubins_RS(gN, visualize)
%Q10D_Q4D_RS Summary of this function goes here
%   Detailed explanation goes here

addpath(genpath('..'))

if nargin < 1
  gN = [11; 11; 15; 9; 15];
end

if nargin < 2
  visualize = true;
end

%% Grid and cost
gMin = [-0.35; -0.35; -pi; -0.25; -3.75];
gMax = [ 0.35;  0.35;  pi;  0.25;  3.75];
sD.grid = createGrid(gMin, gMax, gN, 3);

extraArgs.targets = -(sD.grid.xs{1}.^2 + sD.grid.xs{2}.^2);

%% Dynamical system
aRange = [-0.5; 0.5];
alphaMax = 6;
vOther = 0.1;
wMax = 1.5;
dMax = [0.02; 0.02; 0.2; 0.02];

dims = 1:5;

sD.dynSys = P5D_Dubins_Rel([], aRange, alphaMax, vOther, wMax, dMax, dims);

%% Other parameters
tMax = 15;
dt = 0.1;
tau = 0:dt:tMax;

sD.uMode = 'max';
sD.dMode = 'min';

extraArgs.keepLast =  false;
extraArgs.low_memory = true;

save_name = sprintf('%s_%f', mfilename, now);

if visualize
  extraArgs.visualize = true;
  extraArgs.RS_level = -0.05;
  extraArgs.plotData.plotDims = [1 1 1 0 0];
  extraArgs.plotData.projpt = [0; 0];
  extraArgs.deleteLastPlot = true;
  extraArgs.fig_filename = save_name;
end

extraArgs.stopConverge = true;

data = HJIPDE_solve(extraArgs.targets, tau, sD, 'none', extraArgs);

% deriv = computeGradients(sD.grid, data);

%% Save
save_filename = sprintf('%s.mat', save_name);
% save(save_filename, 'sD', 'data', 'tau', 'deriv', '-v7.3');
save(save_filename, 'sD', 'data', 'tau', '-v7.3');

keyboard
end

