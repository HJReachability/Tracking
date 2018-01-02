function P5D_Dubins_RS(gN, visualize)
%Q10D_Q4D_RS Summary of this function goes here
%   Detailed explanation goes here

addpath(genpath('..'))

if nargin < 1
  gN = [31; 31; 21; 21; 21];
end

if nargin < 2
  visualize = true;
end

%% Grid and cost
gMin = [-0.25; -0.25; -30*pi/180; -0.4; -4];
gMax = [ 0.25;  0.25;  30*pi/180;  0.4;  4];
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

%% Otherparameters
tMax = 10;
dt = 0.5;
tau = 0:dt:tMax;

sD.uMode = 'max';
sD.dMode = 'min';

if visualize
  extraArgs.visualize = true;
  extraArgs.RS_level = -0.05  ;
  extraArgs.plotData.plotDims = [1 1 1 0 0];
  extraArgs.plotData.projpt = [0; 0];
  extraArgs.deleteLastPlot = true;
  extraArgs.keepLast =  true;
end

data = HJIPDE_solve(extraArgs.targets, tau, sD, 'none', extraArgs);

%% Save and output worst value
minData = ones(size(tau));
for i = 1:size(data,5)
  data_i = data(:,:,:,:,i);
  minData(i) = max(data_i(:));
  minData(i)
end

save_filename = sprintf('%s_%f.mat', mfilename, now);
save(save_filename, 'sD', 'data', 'tau', 'minData', '-v7.3');

keyboard
end

