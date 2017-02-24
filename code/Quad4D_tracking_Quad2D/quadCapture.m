function [data, g, tau, runtime]=quadCapture(gN, dt, accuracy, g)
%% Input: grid, target, time
if nargin < 1
  gN = 31;
end

t0 = 0;
tMax = 100;
if nargin < 2
  dt = 1;
end
tau = t0:dt:tMax;

if nargin<3
  accuracy = 'low';
end

if nargin <4
    g_min = [-5; -5; -5; -5];
  g_max = [5; 5; 5; 5];
  g_N = gN*ones(length(g_min),1);
  g = createGrid(g_min,g_max, g_N, [], true);
end

%% make initial data
% ignoreDims = [2,4];
% center = [0 0 0 0];

%data0 = zeros(g.shape);
% for i = 1 : g.dim
%   if(all(i ~= ignoreDims))
%     data0 = data0 + (g.xs{i} - center(i)).^2;
%   end
% end
% data0 = -sqrt(data0);

data0 = -shapeRectangleByCorners(g,[0 -Inf 0 -Inf],[0 Inf 0 Inf]);


%% visualize initial data
f1 = figure(1);
clf
[g02D, data02D] = proj(g, data0, [0 1 0 1], 'min');
h1 = surfc(g02D.xs{1}, g02D.xs{2}, data02D);
figure(1)

%% Input: Problem Parameters
aMax = [3 3]; %acceleration in each direction
aMin = -aMax;

bMax = [1 1];
bMin = -bMax;

dMax = [.1 .1];
dMin = -dMax;

uMax = [bMax(1) aMax(1) bMax(2) aMax(2)];
uMin = [bMin(1) aMin(1) bMin(2) aMin(2)];
uMode = 'max';
dMode = 'min';

%% Input: SchemeDatas


dims = [1:4];
schemeData.dynSys = Quad4D2DCAvoid(zeros(4,1), uMax, uMin, dMax, dMin, dims);
schemeData.uMode = uMode;
schemeData.dMode = dMode;
schemeData.grid = g;
schemeData.accuracy = accuracy;

%% Run
tic;
%extraArgs.visualize = 'true';
%extraArgs.stopInit = [0,0,0,0];
extraArgs.keepLast = true;
%extraArgs.visualize = true;
%extraArgs.deleteLastPlot = true;
% extraArgs.plotData.projpt = 0;
% extraArgs.plotData.plotDims = [1 1 1 0];
extraArgs.stopConverge = 1;
extraArgs.targets = data0;
[data, tau] = ...
  HJIPDE_solve(data0, tau, schemeData, 'none',extraArgs);
runtime = toc;


%% Save 
% if ndims(data) > 4
%   data = min(data,[],5);
% end
% save(['CDC_pl3_abs_target_' num2str(g.N(1)) 'x' num2str(g.N(1)) '_'...
%   num2str(tMax) 'sec_' schemeData.accuracy 'acc_Goal.mat'],...
%   'data','g','tau','runtime','-v7.3');
end
