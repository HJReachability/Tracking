function [vf, runtime]=quadCaptureSplit(gN, dt, tMax, accuracy, targetType, gX, gY)
%% Input: grid, target, time
if nargin < 1
  gN = 71;
end


if nargin < 2
  dt = 1;
end

if nargin < 3
  tMax = 50;
end

t0 = 0;
tau = t0:dt:tMax;

if nargin<4
  accuracy = 'high';
end

if nargin <5
  targetType = 'oneNorm';
end

if nargin <6
  gMinX = [-10; -5];
  gMaxX = [10; 5];
  gMinY = [-10; -5];
  gMaxY = [10; 5];
  g_NX = gN*ones(length(gMinX),1);
  g_NY = gN*ones(length(gMinY),1);
  gX = createGrid(gMinX,gMaxX, g_NX, [], true);
  gY = createGrid(gMinY,gMaxY, g_NY, [], true);
end

%% make initial data


if strcmp(targetType,'oneNorm')
  dataX0 = -shapeRectangleByCorners(gX,[0 -Inf],[0 Inf]);
  dataY0 = -shapeRectangleByCorners(gY,[0 -Inf],[0 Inf]);
  
elseif strcmp(targetType,'quadratic')
  dataX0 = -gX.xs{1}.^2;
  dataY0 = -gY.xs{1}.^2;
end

%% visualize initial data
% f1 = figure(1);
% clf
% subplot(1,2,1)
% hX = surfc(gX.xs{1}, gX.xs{2}, dataX0);
% xlabel('x')
% ylabel('v_x')
% subplot(1,2,2)
% hY = surfc(gY.xs{1}, gY.xs{2}, dataY0);
% xlabel('y')
% ylabel('v_y')
% figure(1)

%% Input: Problem Parameters
aMax = [3 3];
aMin = -aMax;

bMax = [.5 .5];
bMin = -bMax;

dMax = [.1 .1];
dMin = -dMax;

uMax = [bMax(1) aMax(1) bMax(2) aMax(2)];
uMin = [bMin(1) aMin(1) bMin(2) aMin(2)];
uMode = 'max';
dMode = 'min';

%% Input: SchemeDatas
Xdims = 1:2;
Ydims = 3:4;

sD_X.dynSys = Quad4D2DCAvoid(zeros(2,1), uMax, uMin, dMax, dMin, Xdims);
sD_Y.dynSys = Quad4D2DCAvoid(zeros(2,1), uMax, uMin, dMax, dMin, Ydims);

%dims = [1:4];
%schemeData.dynSys = Quad4D2DCAvoid(zeros(4,1), uMax, uMin, dMax, dMin, dims);


sD_X.uMode = uMode;
sD_Y.uMode = uMode;
sD_X.dMode = dMode;
sD_Y.dMode = dMode;

sD_X.grid = gX;
sD_Y.grid = gY;

sD_X.accuracy = accuracy;
sD_Y.accuracy = accuracy;

%% Run
tic;
%extraArgs.visualize = 'true';
%extraArgs.stopInit = [0,0,0,0];
%extraArgs.keepLast = true;
%extraArgs.visualize = true;
%extraArgs.deleteLastPlot = true;
% extraArgs.plotData.projpt = 0;
% extraArgs.plotData.plotDims = [1 1 1 0];
extraArgs.stopConverge = 1;
extraArgs.convergeThreshold = .01;
extraArgs.targets = dataX0;
[dataX, tau] = HJIPDE_solve(dataX0, tau, sD_X, 'none');%, extraArgs);
extraArgs.targets = dataY0;
[dataY, tau] = HJIPDE_solve(dataY0, tau, sD_Y, 'none');%, extraArgs);

runtime = toc;

%% visualize initial data
% f1 = figure(2);
% clf
% subplot(2,2,1)
% hX = surfc(gX.xs{1}, gX.xs{2}, dataX(:,:,end));
% xlabel('x')
% ylabel('v_x')
% subplot(2,2,2)
% [gX1D, dataX1D] = proj(gX, dataX(:,:,end), [0 1], 1);
% plot(gX1D.xs{1},dataX1D)
% xlabel('x')
% ylabel('value')
% subplot(2,2,3)
% hY = surfc(gY.xs{1}, gY.xs{2}, dataY(:,:,end));
% xlabel('y')
% ylabel('v_y')
% subplot(2,2,4)
% [gY1D, dataY1D] = proj(gY, dataY(:,:,end), [0 1], 1);
% plot(gY1D.xs{1},dataY1D)
% xlabel('y')
% ylabel('value')
%% Reconstruct
%     vfs - Self-contained (decoupled) value functions
%              .gs:     cell structure of grids
%              .tau:    common time vector
%              .datas:  cell structure of SC datas (value function look-up
%                       tables)
%              .dims: dimensions of each value function (cell}

vfs.gs = {[gX],[gY]};
vfs.tau = tau;
vfs.datas = {dataX, dataY};
vfs.dims = {[1,2];[3,4]};
vf = reconSC(vfs, [gMinX; gMinY], [gMaxX; gMaxY],'during','min');

%% Save 
% if ndims(data) > 4
%   data = min(data,[],5);
% end
% save(['CDC_pl3_abs_target_' num2str(g.N(1)) 'x' num2str(g.N(1)) '_'...
%   num2str(tMax) 'sec_' schemeData.accuracy 'acc_Goal.mat'],...
%   'data','g','tau','runtime','-v7.3');
end
