function Q8D_Q4D_htc_test(file_name, level)

if nargin < 1
  file_name = '';
end

if nargin < 2
  level = -2;
end

load(file_name)

% Simulation parameters
N = 100;
aMax = 0.9;
dt = 0.025;

start_x = zeros(8,1);
start_x([2 6]) = [-1.5 1.5];

uP = zeros(2,N);
uP(2,:) = aMax * ones(N,1);

% Initial list of tracking error bounds, now in ascending-time order
TEB_ind = length(tau);
TEB_inds = zeros(1,N);
TEB_inds(1) = TEB_ind;
% TEB_list = flip(TEB(1:TEB_ind-1));

% Virtual system
virtQuad = Quad4D(start_x([1 2 5 6]), -aMax*[1;1], aMax*[1;1]);
virtQuad.updateState(uP(:,1), dt);

dynSys = sD.dynSys;

% Tracking system
trueQuad = Quad8D(start_x, dynSys.uMin, dynSys.uMax, dynSys.dMin, ...
  dynSys.dMax, 1:8);

tracking_error = nan(1,N);
tracking_error(1) = max(trueQuad.x([1 5]) - virtQuad.x([1 3]));


for i = 2:N
  % Controller
  u = Q8D_Q4D_htc(rel_sys, trueQuad.x, nextState, sD.grid, deriv, indX, indY);
  
  % add random disturbance to velocity within given bound
  d = dynSys.dMin + rand(2,1).*(dynSys.dMax - dynSys.dMin);
  
  % update state of true vehicle
  trueQuad.updateState(u, dt, [], d);
  
  % Update state of virtual vehicle
  virtQuad.updateState(uP(:,i), dt);
  
  % Get TEB index and list
  [indX, indY, TEB_list] = ...
    Q8D_Q4D_gti(trueQuad.x, nextState, sD.grid, data, level);
end

[tracking_error; TEB_inds]
trueQuad.x
end