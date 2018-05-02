function Q8D_Q4D_RS_vis(g, data, tau, level)

f = figure;
f.Color = 'White';
f.Position = [100 0 600 960];

N = 5;
inds2plot = round(logspace(0, log(length(tau))/log(10), N));

s = cell(N,1);
c = cell(N,1);
c2 = cell(N,1);
colors = winter(N+3);
legKey = [];
legVal = cell(N,1);



% 3D plot showing value function
for ii = 1:N
  [g2D, data2D] = proj(g, data(:,:,:,:,inds2plot(ii)), [0 0 1 1], 'max');
  
  color = colors(ii,:);
  
  subplot(2,1,1)
  s{ii} = surf(g2D.vs{1}, g2D.vs{2}, -data2D);
  s{ii}.EdgeAlpha = (0.9*ii/N).^1;
  s{ii}.FaceColor = color;
  s{ii}.FaceAlpha = (0.9*ii/N).^1;
  
  
  
  hold on
  
  extraArgs.applyLight = false;
  c{ii} = visSetIm(g2D, data2D, color, level, extraArgs);
  c{ii}.LineWidth = 3;
  
  daspect([1 1 0.7])
  
  title('$V(r, T-\tau)$', 'Interpreter', 'LaTeX', 'FontSize', 14)
  xlabel('Velocity', 'FontSize', 14)
  ylabel('Position', 'FontSize', 14)
  zlabel('Value', 'FontSize', 14)
  box on
  view([-70 13])
  
  subplot(2,1,2)
  c2{ii} = visSetIm(g2D, data2D', color, level, extraArgs);
  c2{ii}.LineWidth = 3;
  
  legKey = [legKey c2{ii}];
  legVal{ii} = sprintf('\\tau = %.2f', max(tau) - tau(inds2plot(ii)));
  
  hold on
  
  title('$\mathcal B (r, \tau)$', 'Interpreter', 'LaTeX', 'FontSize', 14)
  xlabel('Position', 'FontSize', 14)
  ylabel('Velocity', 'FontSize', 14)
  grid on
  box on
  axis equal
end

subplot(2,1,1)
surf(g2D.vs{1}, g2D.vs{2}, -level*ones(size(g2D.xs{1})), 'FaceColor', ...
  [0.1 0.1 0.1], 'FaceAlpha', 0.1);

legend(legKey, legVal, 'FontSize', 14)

end