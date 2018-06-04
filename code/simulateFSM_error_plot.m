function simulateFSM_error_plot(trueCar, virt_x, controller, dt)

tau = 0:dt:dt*length(controller);

error = sqrt((virt_x(1, :) - trueCar.xhist(1,:)).^2 + ...
  (virt_x(2, :) - trueCar.xhist(2,:)).^2);

f = figure;
f.Color = 'white';
for i = 1:length(virt_x)-1
  if controller(i)
    color = 'r';
  else
    color = 'b';
  end
  
  plot(tau(i), error(i), '.', 'color', color);
  
  if i == 1
    hold on
  end
end

xlabel('Time (s)')
ylabel('Tracking error')

grid on
box on
xlim([0, max(tau)])
ylim([0, 0.05])

tc = plot(1000, 1000, 'r.');
lc = plot(1000, 1000, 'b.');

legend([tc lc], {'Optimal tracking', 'LQR'})
end