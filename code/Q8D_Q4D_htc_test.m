function Q8D_Q4D_htc_test(data, deriv, sD, tau, level, TEB, visualize)

if nargin < 7
  visualize = false;
end
dynSys = sD.dynSys;

% Simulation parameters
N = 200;
aMax = sD.dynSys.aMax;
aMin = sD.dynSys.aMin;
dt = 0.005;

start_x = zeros(8,1);
% start_x([2 6]) = [-1.5 1.5];

uP = zeros(2,N);
uP(2,:) = aMax(2) * ones(N,1);

% Initial list of tracking error bounds, now in ascending-time order
TEB_ind = length(tau);
% TEB_list = flip(TEB(1:TEB_ind-1));

% Virtual system
virtQuad = Quad4D(start_x([1 2 5 6]), aMin, aMax);
virtQuad.updateState(uP(:,1), dt);

% Tracking system
trueQuad = Quad8D(start_x, dynSys.uMin, dynSys.uMax, dynSys.dMin, ...
  dynSys.dMax, 1:8);

tracking_error = nan(1,N-1);
tracking_error(1) = 0;

deriv_ind.x = length(tau);
deriv_ind.y = length(tau);

TEB_ind_lookup_time = 0;
table_lookup_time = 0;

if visualize
  figure
  colors = lines(N);
  
  plot(trueQuad.x(1), trueQuad.x(5), 'o', 'color', colors(1,:))
  hold on
  drawnow;
end

t = 0;
tau(1) = [];
for i = 2:N
  t = t+dt;
  % Controller
  tic
  u = Q8D_Q4D_htc( ...
    dynSys, sD.uMode, trueQuad.x, virtQuad.x, sD.grid, deriv, deriv_ind);
  table_lookup_time = table_lookup_time + toc;
  
  % add random disturbance to velocity within given bound
  d = dynSys.dMin + rand(2,1).*(dynSys.dMax - dynSys.dMin);
  
  % update state of true vehicle
  trueQuad.updateState(u, dt, [], d);
  
  % Compute tracking error
  tracking_error(i) = max(trueQuad.x([1 5]) - virtQuad.x([1 3]));
  
  % Update state of virtual vehicle (planner computes next state)
  virtQuad.updateState(uP(:,i), dt);
  
  % Get TEB index and list
  if t >= tau(1) - 1e-3
    fprintf('t = %.2f, looking up TEB_ind...', t)
    tic
    [TEB_ind, deriv_ind, TEB_list] = ...
      Q8D_Q4D_gti(trueQuad.x, virtQuad.x, sD.grid, data, level, TEB);
    TEB_ind_lookup_time = TEB_ind_lookup_time + toc;
    
    tau(1) = [];
    
    fprintf(' TEB_ind = %d\n', TEB_ind)
  end
  
  if visualize
    plot(trueQuad.x(1), trueQuad.x(5), 'o', 'color', colors(i-1,:))
    plot(virtQuad.x(1), virtQuad.x(3), '+', 'color', colors(i,:))
    drawnow;
  end
end

disp('Tracking error and TEB inds')
[0:dt:dt*(N-1); tracking_error]
disp('Quadrotor state')
trueQuad.x
fprintf('Table look up time: %.2f, average %.2f per iteration\n', table_lookup_time, table_lookup_time/(N-1))
fprintf('TEB_ind look up time: %.2f, average %.2f per iteration\n', TEB_ind_lookup_time, TEB_ind_lookup_time/(N-1))
end