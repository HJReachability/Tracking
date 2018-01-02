function Q8D_Q4D_RS(gN, visualize)
%Q10D_Q4D_RS Summary of this function goes here
%   Detailed explanation goes here

addpath(genpath('..'))

if nargin < 1
  gN = [41; 41; 25; 21];
end

if nargin < 2
  visualize = true;
end

%% Grid and cost
gMin = [-2; -4; -35*pi/180; -1];
gMax = [ 2;  4;  35*pi/180;  1];
sD.grid = createGrid(gMin, gMax, gN);

extraArgs.targets = -sD.grid.xs{1}.^2;

%% Dynamical system
uMin = [-20/180*pi; -20/180*pi];
uMax = [20/180*pi; 20/180*pi];

aMin = [-0.5; -0.5];
aMax = [0.5; 0.5];

dMax = [0.1; 0.1];
dMin = [-0.1; -0.1];

dims = 1:4;

sD.dynSys = Q8D_Q4D_Rel([], uMin, uMax, aMin, aMax, dMin, dMax, dims);

%% Otherparameters
tMax = 15;
dt = 0.5;
tau = 0:dt:tMax;

sD.uMode = 'max';
sD.dMode = 'min';

if visualize
  extraArgs.visualize = true;
  extraArgs.RS_level = -5;
  extraArgs.plotData.plotDims = [1 1 1 0];
  extraArgs.plotData.projpt = 0;
  extraArgs.deleteLastPlot = true;
end

data = HJIPDE_solve(extraArgs.targets, tau, sD, 'none', extraArgs);

%% Save and output worst value
minData = ones(size(tau));
for i = 1:size(data,5)
  data_i = data(:,:,:,:,i);
  minData(i) = max(data_i(:));
  minData(i)
end

deriv = computeGradients(sD.grid, data);

save_filename = sprintf('%s_%f.mat', mfilename, now);
save(save_filename, 'sD', 'data', 'tau', 'minData', 'deriv', '-v7.3');

keyboard
end

