function [TEB_tot,dMax_tot]=Q10D_Q3D_iterate_control(gN,dt,tMax,extraArgs)
if nargin <1
gN = 45;
end

if nargin <2
dt = 0.01;
end

if nargin<3
tMax = 20;
end

if nargin<4
extraArgs.visualize = 1;
extraArgs.targetType = 'quadratic';
extraArgs.accuracy = 'high';
extraArgs.stopConverge = 1;
extraArgs.convergeThreshold = dt;
end

iterations = 10;
TEB_tot = zeros(1,iterations);
dMax_tot = zeros(1,iterations);

gravity = 9.81;
extraArgs.uMax = [0; 20/180*pi; 0; 20/180*pi; 0; 1.5*gravity];
extraArgs.uMin = [-0; -20/180*pi; -0; -20/180*pi; -0; 0];

for i = 1:iterations
  u = (i/10);

  extraArgs.uMax([1 3 5]) = u;
  extraArgs.uMin([1 3 5]) = -u;
  
  [sD_X, sD_Z, dataX, dataZ, dataX0, dataZ0, tauX, tauZ, TEB] = ...
    Q10D_Q3D_RS(gN, dt, tMax, extraArgs);
  save(['Q10D_Q3D_Rel_u_' num2str(i) '_tenth.mat'],...
    'sD_X', 'sD_Z', 'dataX', 'dataZ', 'dataX0', 'dataZ0', ...
    'tauX', 'tauZ', 'TEB', 'dMax','-v7.3');
  
  TEB_tot(i) = TEB;
  u_tot(i) = u;
end

save('TEB_vs_u.mat','TEB_tot','u_tot')
end