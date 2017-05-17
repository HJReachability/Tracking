function [TEB_tot,u_tot]=Q10D_Q3D_iterate_control(gNX, gNZ,dt,tMax,extraArgs)
if nargin <1
N = 50;
gNX = [N N ceil(N/8) ceil(N/5)];
gNZ = [N N];
end

if nargin <2
dt = 0.1;
end

if nargin<3
tMax = 15;
end

if nargin<4
extraArgs.visualize = 1;
extraArgs.targetType = 'quadratic';
extraArgs.accuracy = 'veryHigh';
extraArgs.stopConverge = 1;
extraArgs.convergeThreshold = dt;
end

iterations = 10;
TEB_tot = zeros(1,iterations);
u_tot = zeros(1,iterations);

gravity = 9.81;
extraArgs.uMax = [0; 20/180*pi; 0; 20/180*pi; 0; 1.5*gravity];
extraArgs.uMin = [-0; -20/180*pi; -0; -20/180*pi; -0; 0];

for i = 10:10+iterations
  u = (i/10);

  extraArgs.uMax([1 3 5]) = u;
  extraArgs.uMin([1 3 5]) = -u;
  
  [sD_X, sD_Z, dataX, dataZ, dataX0, dataZ0, tauX, tauZ, TEB] = ...
    Q10D_Q3D_RS(gNX, gNZ, dt, tMax, extraArgs);
  save(['Q10D_Q3D_Rel_u_' num2str(i) '_tenth.mat'],...
    'sD_X', 'sD_Z', 'dataX', 'dataZ', 'dataX0', 'dataZ0', ...
    'tauX', 'tauZ', 'TEB', 'u','-v7.3');
  
  TEB_tot(i) = TEB;
  u_tot(i) = u;
end

save('TEB_vs_u_10_to_20.mat','TEB_tot','u_tot')
end