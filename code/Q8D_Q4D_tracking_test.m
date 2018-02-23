function Q8D_Q4D_tracking_test(data, deriv, sD, tau)

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
min_level = -0.6;

% Virtual system
virtQuad = Quad4D(start_x([1 2 5 6]), -aMax*[1;1], aMax*[1;1]);


virtQuad.updateState(uP(:,1), dt);

dynSys = sD.dynSys;

% Tracking system
trueQuad = Quad8D(start_x, dynSys.uMin, dynSys.uMax, dynSys.dMin, ...
  dynSys.dMax, 1:8);

uMode = 'max';

XDims = 1:4;
YDims = 5:8;

% Relative dynamics matrix
Q = zeros(8,4);
Q(1,1) = 1;
Q(2,2) = 1;
Q(5,3) = 1;
Q(6,4) = 1;

rel_x = trueQuad.x - Q*virtQuad.x;

tracking_error = nan(1,N);
tracking_error(1) = max(trueQuad.x([1 5]) - virtQuad.x([1 3]));


for i = 2:N
%   i
  % 2. Determine which controller to use, find optimal control
  % get spatial gradients
  deriv_TEB_ind = cell(4,1);
  for k = 1:4
    deriv_TEB_ind{k} = deriv{k}(:,:,:,:,TEB_ind);
  end
  pX = eval_u(sD.grid, deriv_TEB_ind, rel_x(XDims));
  pY = eval_u(sD.grid, deriv_TEB_ind, rel_x(YDims));
  
  % Find optimal control of relative system (no performance control)
  uX = dynSys.optCtrl([], rel_x(XDims), pX, uMode);
  uY = dynSys.optCtrl([], rel_x(YDims), pY, uMode);
  
  u = [uX uY];
  
  % add random disturbance to velocity within given bound
  d = dynSys.dMin + rand(2,1).*(dynSys.dMax - dynSys.dMin);
  
  % update state of true vehicle
  trueQuad.updateState(u, dt, [], d);
  
  % Update state of virtual vehicle
  virtQuad.updateState(uP(:,i), dt);
  
  %% Determine which tracking error bound to start with next (takes about 0.2s)
  rel_x = trueQuad.x - Q*virtQuad.x;
  
  tracking_error(i) = max(trueQuad.x([1 5]) - virtQuad.x([1 3]));
  
  TEB_ind_x = get_TEB_ind(tau, sD, data, rel_x(1:4), min_level);
  TEB_ind_y = get_TEB_ind(tau, sD, data, rel_x(5:8), min_level);
  
%   [TEB_ind_x, values_x] = get_TEB_ind(tau, sD, data, rel_x(1:4), min_level);
%   [TEB_ind_y, values_y] = get_TEB_ind(tau, sD, data, rel_x(5:8), min_level);
  
  TEB_ind = min(TEB_ind_x, TEB_ind_y);
  TEB_inds(i) = TEB_ind;
%   TEB_list = flip(TEB(1:TEB_ind-1));
end

[tracking_error; TEB_inds]
trueQuad.x
end