function Q10D_Q3D_RS_vis(g, data, tau, level)

f = figure;
f.Color = 'White';
f.Position = [100 100 960 600];

N = length(tau);
inds2plot = 1:length(tau);

s = cell(N,1);
c = cell(N,1);
colors = hot(N+10);
legKey = [];
legVal = cell(N,1);

int_font = {'Interpreter', 'LaTeX', 'FontSize', 14};

% 3D plot showing value function
for ii = 1:N
  [g2D, data2D] = proj(g, data(:,:,:,:,inds2plot(ii)), [0 0 1 1], 'min');
  
  color = colors(ii+2,:);
  
  subplot(1,2,1)
  s{ii} = surf(g2D.vs{1}, g2D.vs{2}, data2D');
  s{ii}.EdgeAlpha = (0.9*ii/N).^1;
  s{ii}.FaceColor = color;
  s{ii}.FaceAlpha = (0.9*ii/N).^1;
    
  hold on
  
  extraArgs.applyLight = false;
%   c{ii} = visSetIm(g2D, data2D, color, level, extraArgs);
%   c{ii}.LineWidth = 1;
  
  daspect([0.5 1 0.1])
  xlim([-1 1])
  ylim([-2.5 2.5])
  zlim([0 0.5])
  
  title('$V(r, \tau)$', int_font{:})
  xlabel('$x_r$', int_font{:})
  ylabel('$v_{x,r}$', int_font{:})
  box on
  view([-165 31])
  
  legKey = [legKey s{ii}];
  legVal{ii} = sprintf('$\\tau = %.1f$', tau(inds2plot(ii)));
end

subplot(1,2,1)
surf(g2D.vs{1}, g2D.vs{2}, level*ones(size(g2D.xs{1})), 'FaceColor', ...
  [0.1 0.1 0.1], 'FaceAlpha', 0.1);


legend(legKey, legVal, int_font{:})

subplot(1,2,2)
c2 = visSetIm(g2D, data2D, color, level, extraArgs);
c2.LineWidth = 3;

hold on

title('$\mathcal B_\infty (r)$', int_font{:})
xlabel('$x_r$', int_font{:})
ylabel('$v_{x,r}$', int_font{:})

xlim([-1 1])
ylim([-2.5 2.5])

grid on
box on
axis square


end