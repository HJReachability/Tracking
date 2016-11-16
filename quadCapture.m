function [data,g,tau,runtime] = quadCapture(g, radius, tau, accuracy)
%% Input: grid, target, time

if nargin <1
  g_min = [-radius-2; -5; -radius-2; -5];
  g_max = [radius+2; 5; radius+2; 5];
  g_N = 35*ones(length(g_min),1);
  g = createGrid(g_min,g_max, g_N, [], true);
end

if nargin <2
  radius = 2;
%   dataMin = [-10 -inf -10 -inf];
%   dataMax = [10 inf 10 inf];
end
  %data0 = shapeRectangleByCorners(g, ...
  %  dataMin, dataMax);
  data0 = shapeCylinder(g,[2,4],[0,0,0,0],radius);
  data0 = shapeComplement(data0);

if nargin <3
  % time vector
  t0 = 0;
  tMax = 30;
  dt = 0.025;
  tau = t0:dt:tMax;
end

if nargin<4
  accuracy = 'high';
end
% If intermediate results are not needed, use tau = [t0 tMax];

%% Input: Problem Parameters
aMax = 2;
axMax = aMax;
ayMax = aMax;
axMin = -aMax;
ayMin = -aMax;
uMax = 0.5;
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
extraArgs.stopInit = [0,0,0,0];
extraArgs.stopConverge = 1;
[data, tau] = ...
  HJIPDE_solve(data0, tau, schemeData, 'zero',extraArgs,0);
runtime = toc;


%% Save 
% if ndims(data) > 4
%   data = min(data,[],5);
% end
% save(['CDC_pl3_abs_target_' num2str(g.N(1)) 'x' num2str(g.N(1)) '_'...
%   num2str(tMax) 'sec_' schemeData.accuracy 'acc_Goal.mat'],...
%   'data','g','tau','runtime','-v7.3');
end
