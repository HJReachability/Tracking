function [data,g,tau,runtime] = quadCapture(g, tau, accuracy)
%% Input: grid, target, time
% if nargin <1
%   radius = 2;
% %   dataMin = [-10 -inf -10 -inf];
% %   dataMax = [10 inf 10 inf];
% end

if nargin <2
    g_min = [-5; -5; -5; -5];
  g_max = [5; 5; 5; 5];
%   g_min = [-10; -10; -10; -10];
%   g_max = [10; 10; 10; 10];
  g_N = 35*ones(length(g_min),1);
  g = createGrid(g_min,g_max, g_N, [], true);
end


  %data0 = shapeRectangleByCorners(g, ...
  %  dataMin, dataMax);
%   data0 = shapeCylinder(g,[2,4],[0,0,0,0],radius);
%   data0 = shapeComplement(data0);
%   data0 = data0 - max(data0(:)); %make maximum value = 0
%% make initial data
ignoreDims = [2,4];
center = [0 0 0 0];

data0 = zeros(g.shape);
for i = 1 : g.dim
  if(all(i ~= ignoreDims))
    data0 = data0 + (g.xs{i} - center(i)).^2;
  end
end
data0 = -sqrt(data0);

%% time
if nargin <3
  % time vector
  t0 = 0;
  tMax = .25;
  dt = 0.025;
  tau = t0:dt:tMax;
end

if nargin<4
  accuracy = 'medium';
end
% If intermediate results are not needed, use tau = [t0 tMax];

%% Input: Problem Parameters
aMax = 4;
axMax = aMax;
ayMax = aMax;
axMin = -aMax;
ayMin = -aMax;
uMax = 2;
uxMax = uMax;
uyMax = uMax;
uxMin = -uMax;
uyMin = -uMax;


%% Input: SchemeDatas
schemeData.grid = g;
schemeData.axMax = axMax;
schemeData.axMin = axMin;
schemeData.ayMax = ayMax;
schemeData.ayMin = ayMin;
schemeData.uxMax = uxMax;
schemeData.uyMax = uyMax;
schemeData.uxMin = uxMin;
schemeData.uyMin = uyMin;
schemeData.hamFunc = @quadCaptureHam;
schemeData.partialFunc = @quadCapturePartial;
schemeData.accuracy = accuracy;

%% Run
tic;
%extraArgs.visualize = 'true';
%extraArgs.stopInit = [0,0,0,0];
extraArgs.keepLast = true;
extraArgs.visualize = true;
extraArgs.deleteLastPlot = true;
extraArgs.plotData.projpt = 0;
extraArgs.plotData.plotDims = [1 1 1 0];
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
