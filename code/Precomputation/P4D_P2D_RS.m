function [trackingRadius, sD, data0, data] = P4D_P2D_RS()
% FaSTrack
wmax = 2*pi;
vxmax = 1;
vymax = 1;
dMax = [0.8, 0.8]; %%%%% making 2 disturbances
aRange = [-1.5, 1.5];

%% Computing Tracking Error Bound (TEB)
TEBgN = [55; 55; 25; 25];
TEBgMin = [-3; -3; -pi; -2];
TEBgMax = [ 3;  3;  pi; 2];
sD.grid = createGrid(TEBgMin, TEBgMax, TEBgN, 3);
data0 = sD.grid.xs{1}.^2 + sD.grid.xs{2}.^2;
%extraArgs.obstacles = -data0; 

% Visualization
[TEBg2D, data02D] = proj(sD.grid,data0,[0 0 1 1],[0 0]);
figure(1)
clf
subplot(1,2,1)
surf(TEBg2D.xs{1}, TEBg2D.xs{2}, sqrt(data02D))
hold on

sD.dynSys = P4D_Q2D_Rel([0; 0; 0; 0], -wmax, wmax, vxmax, vymax, ...
   -dMax, dMax, aRange(1), aRange(2), [1 2 3 4]);
%sD.dynSys = P4D_Q2D_Rel([0; 0; 0; 0], -2, 2, vxmax, vymax, ...
%    -dMax, dMax, aRange(1), aRange(2), [1 2 3 4]);
sD.uMode = 'min';
sD.dMode = 'max';
sD.accuracy = 'low';

f = figure(2);
%set(f, 'Position', [400 400 450 400]); %%%%% removing this for debugging
extraArgs.visualize = true;
extraArgs.RS_level = 2; 
extraArgs.fig_num = 2;
extraArgs.plotData.plotDims = [1 1 0 0];
extraArgs.plotData.projpt = [0 0];
extraArgs.deleteLastPlot = false;

dt = 0.1;
tMax = 1;
tau = 0:dt:tMax;
extraArgs.stopConverge = true;
extraArgs.convergeThreshold = dt;
extraArgs.keepLast = true;

% solve backwards reachable set
% instead of none, do max_data0 - no obstacles or max over time
[data, tau] = HJIPDE_solve(data0, tau, sD, 'maxWithTarget', extraArgs); 

% largest cost along all time (along the entire 5th dimension which is
% time)
%data = max(data,[],5); 

figure(1)
subplot(1,2,2)
[TEBg2D, data2D] = proj(sD.grid, data, [0 0 1 1], [0 0]);
s = surf(TEBg2D.xs{1}, TEBg2D.xs{2}, sqrt(data2D));
  
%Level set for tracking error bound
lev = min(min(s.ZData));


% More visualization
f = figure(3);
clf
%set(f, 'Position', [360 278 560 420]); %%%%% removing for debugging
alpha = .2;
small = .05;
 
levels = [lev, lev + .2, lev + .4];
%levels = [.5, .75, 1];  
%levels = [2.6, 3, 3.3];
  
[g3D, data3D] = proj(sD.grid,data,[0 0 0 1], .5);
[~, data03D] = proj(sD.grid,data0,[0 0 0 1], .5);
%plotlevel = min(data3D(:));  


for i = 1:3
    subplot(2,3,i)
    h0 = visSetIm(g3D, sqrt(data03D), 'blue', levels(i)+small);
    h0.FaceAlpha = alpha;
    hold on
    h = visSetIm(g3D, sqrt(data3D), 'red', levels(i));
    axis([-levels(3)-3*small levels(3)+3*small ...
      -levels(3)-3*small levels(3)+3*small -pi pi])
  if i == 2
    title(['t = ' num2str(tau(end)) ' s'])
  end
    axis square
end
  
for i = 4:6
    subplot(2,3,i)
    h0 = visSetIm(TEBg2D, sqrt(data02D), 'blue', levels(i-3)+small);
    hold on
    h = visSetIm(TEBg2D, sqrt(data2D), 'red', levels(i-3));
    axis([-levels(3)-small levels(3)+small ...
      -levels(3)-small levels(3)+small])
    title(['R = ' num2str(levels(i-3))])
    axis square
          
set(gcf,'Color','white')
end


teb = sqrt(min(data(:))) + 0.03;
lev = lev + 0.03;
trackingRadius = lev;

figure(4)
clf
h0 = visSetIm(TEBg2D, sqrt(data02D), 'blue', lev+small);
hold on
h = visSetIm(TEBg2D, sqrt(data2D), 'red', lev);
title(['Tracking Error Bound R = ' num2str(lev)])
axis square


end