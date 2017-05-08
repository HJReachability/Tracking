function [TEB_tot,dMax_tot]=Q10D_Q3D_iterate_disturbances(gN,dt,tMax,extraArgs)
if nargin <1
gN = 20;
end

if nargin <2
dt = 0.1;
end

if nargin<3
tMax = 20;
end

if nargin<4
extraArgs.visualize = 0;
extraArgs.targetType = 'quadratic';
extraArgs.accuracy = 'low';
extraArgs.stopConverge = 1;
extraArgs.convergeThreshold = dt;
end

iterations = 10;
TEB_tot = zeros(1,iterations);
dMax_tot = zeros(1,iterations);

for i = 1:iterations
  dMax = (i/10);
  extraArgs.dMax = ones(4,1)*dMax;
  
  [sD_X, sD_Z, dataX, dataZ, dataX0, dataZ0, tauX, tauZ, TEB] = ...
    Q10D_Q3D_RS(gN, dt, tMax, extraArgs);
  save(['Q10D_Q3D_Rel_dMax_' num2str(i) '_tenth.mat'],...
    'sD_X', 'sD_Z', 'dataX', 'dataZ', 'dataX0', 'dataZ0', ...
    'tauX', 'tauZ', 'TEB', 'dMax','-v7.3');
  
  TEB_tot(i) = TEB;
  dMax_tot(i) = dMax;
end

save('TEB_vs_dMax.mat','TEB_tot','dMax_tot')
end