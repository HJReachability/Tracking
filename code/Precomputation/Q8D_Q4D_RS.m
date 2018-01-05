function Q8D_Q4D_RS(A, gN, visualize)
%Q10D_Q4D_RS Summary of this function goes here
%   Detailed explanation goes here

addpath(genpath('..'))

if nargin < 2
  gN = [41; 41; 25; 21];
end

if nargin < 3
  visualize = true;
end

%% Grid and cost
gMin = [-2; -4; -35*pi/180; -2*pi];
gMax = [ 2;  4;  35*pi/180;  2*pi];
sD.grid = createGrid(gMin, gMax, gN);

extraArgs.targets = -sD.grid.xs{1}.^2;

%% Dynamical system
uMin = [-20/180*pi; -20/180*pi];
uMax = [20/180*pi; 20/180*pi];

aMin = -A*ones(2,1);
aMax = A*ones(2,1);

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

save_filename = sprintf('%s_%f_%f.mat', mfilename, A, now);
save(save_filename, 'sD', 'data', 'tau', 'minData', 'deriv', '-v7.3');

end

