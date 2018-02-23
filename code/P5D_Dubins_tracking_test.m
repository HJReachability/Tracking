function P5D_Dubins_tracking_test(deriv, sD)

% Simulation parameters
dynSys = sD.dynSys;

N = 100;
dt = 0.025;

start_x = zeros(5,1);
start_x(4) = 0.1; % Initial speed

uP = zeros(1,N); % Planner

% Virtual system / planner (has no disturbance)
dCar = DubinsCar(start_x(1:3), dynSys.wMax, dynSys.vOther);
dCar.updateState(uP(1), dt);

% Tracking system
trueCar = Plane5D(start_x, dynSys.alphaMax, dynSys.aRange, dynSys.dMax);

uMode = 'max';

% Relative dynamics matrix
Q = zeros(5,3);
Q(1,1) = 1;
Q(2,2) = 1;
Q(3,3) = 1;

rel_x = trueCar.x - Q*dCar.x;

tracking_error = nan(1,N);
tracking_error(1) = norm(trueCar.x(1:2) - dCar.x(1:2));

for i = 2:N
  % 2. Determine which controller to use, find optimal control
  % get spatial gradients
  p = eval_u(sD.grid, deriv, rel_x);
  
  % Find optimal control of relative system (no performance control)
  u = dynSys.optCtrl([], rel_x, p, uMode);
  
  % add random disturbance to velocity within given bound
  d = -dynSys.dMax + 2*rand(4,1).*dynSys.dMax;
  
  % update state of true vehicle
  trueCar.updateState(u, dt, [], d);
  
  % Update state of virtual vehicle
  dCar.updateState(uP(i), dt);
  
  %% Determine which tracking error bound to start with next (takes about 0.2s)
  rel_x = trueCar.x - Q*dCar.x;
  
  tracking_error(i) = norm(trueCar.x(1:2) - dCar.x(1:2));
end

trueCar.xhist
tracking_error
end