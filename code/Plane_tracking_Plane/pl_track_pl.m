function pl_track_pl(save_png)
% computeRTTRS(obj, vR, wR, trackingRadius, save_png)
%     Computes the robust trajectory tracking reachable set and updates the SPPP
%     object with the RTTRS file name
%
% Inputs:
%     obj - SPP problem object
%     vR - reserved vehicle speed
%     wR - reserved angular acceleration
%     tR - tracking radius
%     save_png - set to true to save figures
%
% Output:
%     SPPP - SPP problem object updated with RTTRS file name

if nargin < 1
  save_png = true;
end

%% Virtual Plane
wMax_virt = 0.6;
vrange_virt = [2.5 3.5];
virt_pl = Plane([0;0;0], wMax_virt, vrange_virt);

%% Real Plane
wMax = pi/2;      % rad/s
vrange = [0.1 5]; % m/s
dMax = [1 0];     % m/s

real_pl = Plane([0;0;0], wMax, vrange, dMax);

% Pursuit evasion dynamical system
dynSys = PlaneCAvoid({virt_pl; real_pl});

% Grid
L = 10;
grid_min = [-L; -L; -pi]; % Lower corner of computation domain
grid_max = [L; L; pi];    % Upper corner of computation domain
N = [101; 101; 101]; 
g = createGrid(grid_min, grid_max, N, 3);

% Track trajectory for up to this time
tMax = 10; % for SPPwIntruderRTT method 2
dt = 0.025;
tau = 0:dt:tMax;

% Initial conditions
data0 = -sqrt(g.xs{1}.^2 + g.xs{2}.^2);
schemeData.dynSys = dynSys;
schemeData.grid = g;
schemeData.uMode = 'max';
schemeData.dMode = 'min';
extraArgs.visualize = true;
extraArgs.RS_level = -0.9*L;
extraArgs.deleteLastPlot = true;
extraArgs.keepLast = true;

if save_png
  folder = sprintf('%s', mfilename);
  system(sprintf('mkdir %s', folder));
  
  extraArgs.fig_filename = sprintf('%s/', folder);
end

% Compute
data = HJIPDE_solve(data0, tau, schemeData, 'zero', extraArgs);

save(sprintf('%s/set.mat', folder), 'g', 'data', '-v7.3');
end