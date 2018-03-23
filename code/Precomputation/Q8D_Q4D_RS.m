function Q8D_Q4D_RS(A, D, gN, visualize)
%Q10D_Q4D_RS Summary of this function goes here
%   Detailed explanation goes here

addpath(genpath('..'))

if nargin < 3
  gN = [41; 41; 25; 21];
end

if nargin < 4
  visualize = true;
end

save_name = sprintf('%s_%.2f_%.4f_', mfilename, A, rem(now,1));

%% Grid and cost
gMin = [-2; -2; -35*pi/180; -2*pi];
gMax = [ 2;  2;  35*pi/180;  2*pi];
sD.grid = createGrid(gMin, gMax, gN);

extraArgs.targets = -sqrt(sD.grid.xs{1}.^2 + sD.grid.xs{2}.^2 / 4);
extraArgs.low_memory = true;
%% Dynamical system
uMin = [-20/180*pi; -20/180*pi];
uMax = [20/180*pi; 20/180*pi];

aMin = -A*ones(2,1);
aMax = A*ones(2,1);

dMax = [D; D];
dMin = [-D; -D];

dims = 1:4;

sD.dynSys = Q8D_Q4D_Rel([], uMin, uMax, aMin, aMax, dMin, dMax, dims);

%% Otherparameters
tMax = 15;
dt = 0.2;
tau = 0:dt:tMax;

sD.uMode = 'max';
sD.dMode = 'min';

if visualize
  extraArgs.visualize = true;
  extraArgs.RS_level = -0.25;
  extraArgs.plotData.plotDims = [1 1 1 0];
  extraArgs.plotData.projpt = 0;
  extraArgs.deleteLastPlot = true;
  extraArgs.fig_filename = save_name;
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

save_filename = sprintf('%s.mat', save_name);
save(save_filename, 'sD', 'data', 'tau', 'minData', 'deriv', '-v7.3');

end

