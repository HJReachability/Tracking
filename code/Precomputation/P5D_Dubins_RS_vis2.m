function P5D_Dubins_RS_vis2(g, data, level)
% P5D_Dubins_RS_vis(g, data, level)
%
% Creates figure to visualize TEB for P5D_Dubins example for the journal 
% paper (example involving 5D car tracking 3D car)

f = figure;
f.Color = 'White';
f.Position = [100 100 800 600];

% Slices to plot
theta_slice = pi/2;
as = '\pi/2';

% Formatting
int_font = {'Interpreter', 'LaTeX', 'FontSize', 14};
color = 'm';

L = 0.1;

% Project 5D value function into 3D and 2D
[g3D, data3D] = proj(g, data, [0 0 0 1 1], 'max');

[g2D, data2D] = proj(g3D, data3D, [0 0 1], theta_slice, [151; 151]);




%% Right: 3D TEB
subplot(2, 2, [2 4])

% TEB
c1 = visSetIm(g3D, data3D, 'g', level);
c1.FaceAlpha = 0.8;
hold on

% Horizon planes showing slices

horiz_plane_FaceAlpha = 0.5;

horiz_plane = theta_slice*ones(g2D.N');

surf(g2D.vs{1}, g2D.vs{2}, horiz_plane, 'FaceColor', color, ...
  'FaceAlpha', horiz_plane_FaceAlpha, 'EdgeColor', 'none');


% Formatting
xlim([-L L])
ylim([-L L])
zlim([-pi pi])
axis square
box on
grid on

title("$\mathcal B_{p, \infty}$ (TEB)", int_font{:})
xlabel('$x_r$', int_font{:})
ylabel('$y_r$', int_font{:})
zlabel('$\theta_r$', int_font{:})


%% Right top and Left bottom: Plot value function of 2D slice
  subplot(2, 2, 1)
  
  % Value function
  c1a = surf(g2D.vs{1}, g2D.vs{2}, -data2D', 'FaceColor', color, ...
    'FaceAlpha', 0.9, 'EdgeColor', 'none');
  hold on
  
  data2D_0 = (g2D.xs{1}).^2 + (g2D.xs{2}).^2;

  c1b = surf(g2D.vs{1}, g2D.vs{2}, data2D_0, 'FaceColor', 'b', ...
    'FaceAlpha', 0.2, 'EdgeColor', 'b');
  
  horiz_plane = -level*ones(g2D.N');
  surf(g2D.vs{1}, g2D.vs{2}, horiz_plane, 'FaceColor', 'k', 'FaceAlpha', ...
    0.5, 'EdgeColor', 'none');
  
  % TEB
  c23 = visSetIm(g2D, data2D, color, level);
  c23.LineWidth = 3;
  
  % Formatting
  xlim([-L L])
  ylim([-L L])
  zlim([0 0.1*L])
  box on
  grid on
  
  title_str = ...
    sprintf('Projected slices at $\\theta_r = %s$', as);
  title(title_str, int_font{:})
  xlabel('$x_r$', int_font{:})
  ylabel('$y_r$', int_font{:})
  zlabel('Value', int_font{:})

  legend([c1a c1b], {'$V_\infty(r)$', '$l(r)$'}, int_font{:}, 'Autoupdate', 'off')
  c1b.FaceColor = 'none';
%% Left bottom: TEB
subplot(2, 2, 3)

% TEB

c3 = visSetIm(g2D, data2D, color, level);
c3.LineWidth = 3;
hold on
  
% Formatting
xlim([-L L])
ylim([-L L])
box on
grid on

title("Slices of $\mathcal B_{e, \infty}$", int_font{:})
xlabel('$x_r$', int_font{:})
ylabel('$y_r$', int_font{:})
end