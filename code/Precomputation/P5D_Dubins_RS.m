function P5D_Dubins_RS(gN, visualize)
%Q10D_Q4D_RS Summary of this function goes here
%   Detailed explanation goes here

addpath(genpath('..'))

if nargin < 1
  gN = [35; 35; 35; 35; 35];
end

if nargin < 2
  visualize = true;
end

%% Grid and cost
gMin = [-0.25; -0.25; -90*pi/180; -0.4; -6];
gMax = [ 0.25;  0.25;  90*pi/180;  0.4;  6];
sD.grid = createGrid(gMin, gMax, gN);

extraArgs.targets = -(sD.grid.xs{1}.^2 + sD.grid.xs{2}.^2);

%% Dynamical system
aRange = [-0.25; 0.25];
alphaMax = 5;
vOther = 0.1;
wMax = 2;
dMax = [0.02; 0.02; 0.2; 0.02];

dims = 1:5;

sD.dynSys = P5D_Dubins_Rel([], aRange, alphaMax, vOther, wMax, dMax, dims);

%% Other parameters
tMax = 10;
dt = 0.01;
tau = 0:dt:tMax;

sD.uMode = 'max';
sD.dMode = 'min';

extraArgs.keepLast =  true;
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

data = HJIPDE_solve(extraArgs.targets, tau, sD, 'none', extraArgs);

deriv = computeGradients(sD.grid, data);

%% Save and output worst value
minData = ones(size(tau));
for i = 1:size(data,5)
  data_i = data(:,:,:,:,i);
  minData(i) = max(data_i(:));
  minData(i)
end

save_filename = sprintf('%s.mat', save_name);
save(save_filename, 'sD', 'data', 'tau', 'minData', 'deriv', '-v7.3');

keyboard
end

