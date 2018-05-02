function Q8D_Q4D_RS_vis(g, data, tau, level)

f = figure;
f.Color = 'White';
f.Position = [100 100 800 600];

N = 5;
inds2plot = round(logspace(0, log(length(tau))/log(10), N));

s = cell(N,1);
c = cell(N,1);
colors = hot(N+3);
legKey = [];
legVal = cell(N,1);



% 3D plot showing value function
for ii = 1:N
  [g2D, data2D] = proj(g, data(:,:,:,:,inds2plot(ii)), [0 0 1 1], 'max');
  
  color = colors(ii,:);
  s{ii} = surf(g2D.vs{1}, g2D.vs{2}, -data2D);
%   s{ii}.EdgeColor = 'none';
  s{ii}.EdgeAlpha = (0.9*ii/N).^1;
  s{ii}.FaceColor = color;
  s{ii}.FaceAlpha = (0.9*ii/N).^1;
  
  legKey = [legKey s{ii}];
  legVal{ii} = sprintf('t = %.2f', max(tau) - tau(inds2plot(ii)));
  hold on
  
  extraArgs.applyLight = false;
  c{ii} = visSetIm(g2D, data2D, color, level, extraArgs);
  c{ii}.LineWidth = 3;
%   c{ii}.Visible = 'off';
%   
%   contourZ = -level*ones(length(c{ii}.ContourMatrix(1,2:end)), 1);
%   plot3(c{ii}.ContourMatrix(1,2:end), c{ii}.ContourMatrix(2,2:end), ...
%     contourZ, 'color', colors(ii,:));

end


daspect([1 1 0.7])

legend(legKey, legVal)

xlabel('Velocity', 'FontSize', 14)
ylabel('Position', 'FontSize', 14)
zlabel('Value', 'FontSize', 14)
