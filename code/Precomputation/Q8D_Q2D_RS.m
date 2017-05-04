function [data,tau,sD,teb]=Q8D_Q2D_RS(gN, visualize)
%Q10D_Q4D_RS Summary of this function goes here
%   Detailed explanation goes here

if nargin < 1
  gN = [25; 25; 15];
end

if nargin < 2
  visualize = true;
end

%% Grid and cost
gMin = [-5; -5; -pi];
gMax = [ 5;  5;  pi];
sD.grid = createGrid(gMin, gMax, gN,3);

data0 = sD.grid.xs{1}.^2;
extraArgs.obstacles = -data0;

if visualize
figure(1)
clf
subplot(1,2,1)
[g1D, data1D] = proj(sD.grid, data0, [0 1 1 1], 'min');
data1D = sqrt(data1D);
vfs.gs = {g1D,g1D};
range_lower = [g1D.min, g1D.min];
range_upper = [g1D.max, g1D.max];
vfs.tau = 0;
vfs.datas = {data1D, data1D};
vfs.dims = {1,2};
vf = reconSC(vfs, range_lower, range_upper,0);
surf(vf.g.xs{1}, vf.g.xs{2}, vf.data)
end

%% Dynamical system
uMin = [-20/180*pi; -20/180*pi];
uMax = [20/180*pi; 20/180*pi];

pMin = [-1; -1];
pMax = [1; 1];

dMax = [0.1; 0.1];
dMin = [-0.1; -0.1];

dims = 1:4;

sD.dynSys = Q8D_Q2D_Rel([], uMin, uMax, pMin, pMax, dMin, dMax, dims);

%% Otherparameters
tMax = 10;
dt = 0.1;
tau = 0:dt:tMax;

sD.uMode = 'min';
sD.dMode = 'max';

if visualize
  extraArgs.visualize = true;
  extraArgs.RS_level = 5;
  extraArgs.plotData.plotDims = [1 1 1 0];
  extraArgs.plotData.projpt = 0;
  extraArgs.deleteLastPlot = true;
end

extraArgs.stopConverge = true;
extraArgs.convergeThreshold = 0.5*dt;
[data, tau] = HJIPDE_solve(data0, tau, sD, 'none', extraArgs);

% max over time
data = max(data,[],5);

if visualize
figure(1)
subplot(1,2,2)
[g1D, data1D] = proj(sD.grid, data, [0 1 1 1], 'min');
data1D = sqrt(data1D);
vfs.gs = {g1D,g1D};
range_lower = [g1D.min, g1D.min];
range_upper = [g1D.max, g1D.max];
vfs.tau = 0;
vfs.datas = {data1D, data1D};
vfs.dims = {1,2};
vf = reconSC(vfs, range_lower, range_upper,0);
surf(vf.g.xs{1}, vf.g.xs{2}, vf.data)
end

%tracking error bound
teb = sqrt(min(data(:)))
%% Save and output worst value
save(sprintf('%s_%f.mat', mfilename, now), 'sD', 'data', '-v7.3');

% for i = 1:size(data,5)
%   data_i = data(:,:,:,:,i);
%   max(data_i(:))
% end

%keyboard
end

