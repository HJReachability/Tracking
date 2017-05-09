function [TEB_tot,dMax_tot]=Q10D_Q3D_iterate_disturbances(gNX, gNZ,dt,tMax,extraArgs)
if nargin <1
N = 50;
gNX = [N N ceil(N/8) ceil(N/5)];
gNZ = [N N];
end

if nargin <3
dt = 0.1;
end

if nargin<4
tMax = 15;
end

if nargin<4
extraArgs.visualize = 0;
extraArgs.targetType = 'quadratic';
extraArgs.accuracy = 'veryHigh';
extraArgs.stopConverge = 1;
extraArgs.convergeThreshold = dt;
extraArgs.quiet = 0;
end

iterations = 11;
TEB_tot = zeros(1,iterations);
dMax_tot = zeros(1,iterations);

for i = 1:iterations
  dMax = ((i-1)/10);
  extraArgs.dMax = ones(4,1)*dMax;
  
  [sD_X, sD_Z, dataX, dataZ, dataX0, dataZ0, tauX, tauZ, TEB] = ...
    Q10D_Q3D_RS(gNX, gNZ, dt, tMax, extraArgs);
  save(['Q10D_Q3D_Rel_dMax_' num2str(i-1) '_tenth.mat'],...
    'sD_X', 'sD_Z', 'dataX', 'dataZ', 'dataX0', 'dataZ0', ...
    'tauX', 'tauZ', 'TEB', 'dMax','-v7.3');
  disp(['Q10D_Q3D_Rel_dMax_' num2str(i) '_tenth is done'])
  
  TEB_tot(i) = TEB;
  dMax_tot(i) = dMax;
end

save('TEB_vs_dMax.mat','TEB_tot','dMax_tot')
end