function P5D_Dubins_htc_test(deriv, sD)

% Simulation parameters
dynSys = sD.dynSys;

N = 50000;
dt = 0.001;

start_x = zeros(5,1);
start_x(4) = 0.1; % Initial speed

% uP = linspace(0,1,N); % Planner
% uP = zeros(1,N);
uP = 0.1*ones(1,N);

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

tracking_error = nan(1,N);
tracking_error(1) = norm(trueCar.x(1:2) - dCar.x(1:2));

for i = 2:N
  u = P5D_Dubins_htc(dynSys, uMode, trueCar.x, dCar.x, sD.grid, deriv);
  
  % add random disturbance to velocity within given bound
  d = -dynSys.dMax + 2*rand(4,1).*dynSys.dMax;
  
  % update state of true vehicle
  trueCar.updateState(u, dt, [], d);
  
  % Update state of virtual vehicle
  dCar.updateState(uP(i), dt);

  tracking_error(i) = norm(trueCar.x(1:2) - dCar.x(1:2));
end

f = figure;
f.Color = 'white';
f.Position = [100 100 960 600];
subplot(1,2,1)
plot(trueCar.xhist(1,:), trueCar.xhist(2,:), 'b.-')

subplot(1,2,2)
plot(dt:dt:N*dt, tracking_error)

% trueCar.xhist
% tracking_error

end