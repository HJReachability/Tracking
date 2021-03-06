function [data, g, tau, runtime]=plane4D2DCapture(gN, dt, tMax, accuracy, g)
%% Input: grid, target, time
if nargin < 1
  gN = 31;
end

t0 = 0;

if nargin < 2
  dt = 1;
end

if nargin <3 
  tMax = 50;
end

tau = t0:dt:tMax;

if nargin<4
  accuracy = 'low';
end

if nargin <5
    g_min = [-10; -10; -5; -pi];
  g_max = [10; 10; 5; pi];
  g_N = gN*ones(length(g_min),1);
  g = createGrid(g_min,g_max, g_N, 4, true);
end

%% make initial data
data0 = -sqrt(g.xs{1}.^2 + g.xs{2}.^2);
%data0 = -shapeRectangleByCorners(g,[0 0 -Inf -Inf],[0 0 Inf Inf]);


%% visualize initial data
f1 = figure(1);
clf
[g02D, data02D] = proj(g, data0, [0 0 1 1], 'min');
h1 = surfc(g02D.xs{1}, g02D.xs{2}, data02D);
figure(1)

%% Input: Problem Parameters
aMax = [3 pi/2]; %acceleration, rotational velocity
aMin = -aMax;

bMax = [.5 .5];
bMin = -bMax;

dMax = [.1 .1];
dMin = -dMax;

uMax = [bMax(1) bMax(2) aMax(1) aMax(2)];
uMin = [bMin(1) bMin(2) aMin(1) aMin(2)];
uMode = 'max';
dMode = 'min';

%% Input: SchemeDatas


dims = [1:4];
schemeData.dynSys = Plane4D2DCAvoid(zeros(4,1), uMax, uMin, dMax, dMin, dims);
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
