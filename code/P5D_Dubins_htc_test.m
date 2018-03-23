function P5D_Dubins_htc_test(deriv, sD)

% Simulation parameters
dynSys = sD.dynSys;

N = 5000;
dt = 0.001;

start_x = zeros(5,1);
start_x(4) = 0.1; % Initial speed

% uP = linspace(0,1,N); % Planner
% uP = zeros(1,N);
uP = 2*ones(1,N);
% uP(floor(N/2):end) = 0;

% Virtual system / planner (has no disturbance)
dCar = DubinsCar(start_x(1:3), dynSys.wMax, dynSys.vOther);
dCar.updateState(uP(1), dt);

% Tracking system
trueCar = Plane5D(start_x, dynSys.alphaMax, dynSys.aRange, dynSys.dMax);

uMode = 'max';

tracking_error = nan(1,N);
tracking_error(1) = norm(trueCar.x(1:2) - dCar.x(1:2));

f = figure;
f.Color = 'white';
f.Position = [100 100 1260 540];

plot_pd = 100;
for i = 2:N
  if ~mod(i, plot_pd)
    fprintf('Iteration %d...\n', i)
    if i <= plot_pd
      subplot(1,2,1)
      title('Trajectory')
      pt_skip = ceil(plot_pd/10);
      true_h = plot(trueCar.xhist(1,1:pt_skip:end), ...
        trueCar.xhist(2,1:pt_skip:end), 'b.-');
      hold on
      d_h = plot(dCar.xhist(1,1:pt_skip:end), dCar.xhist(2,1:pt_skip:end), ...
        'ro-');
      axis equal
      
      subplot(1,2,2)
      title('Tracking error')
      t_h = plot(dt:dt:i*dt, tracking_error(1:i));
    else
      true_h.XData = trueCar.xhist(1,1:pt_skip:end);
      true_h.YData = trueCar.xhist(2,1:pt_skip:end);
      d_h.XData = dCar.xhist(1,1:pt_skip:end);
      d_h.YData = dCar.xhist(2,1:pt_skip:end);
      t_h.XData = dt:dt:i*dt;
      t_h.YData = tracking_error(1:i);
    end
    drawnow;
    
    disp(u)
  end  
  u = P5D_Dubins_htc(dynSys, uMode, trueCar.x, dCar.x, sD.grid, deriv);
  
  % add random disturbance to velocity within given bound
  d = -dynSys.dMax + 2*rand(4,1).*dynSys.dMax;
  
  % update state of true vehicle
  trueCar.updateState(u, dt, [], d);
  
  % Update state of virtual vehicle
  dCar.updateState(uP(i), dt);
  
  tracking_error(i) = norm(trueCar.x(1:2) - dCar.x(1:2));
end




% trueCar.xhist
% tracking_error

end
