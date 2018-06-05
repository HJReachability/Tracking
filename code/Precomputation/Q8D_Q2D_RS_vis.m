function Q8D_Q2D_RS_vis(gX, dataX, gZ, dataZ, level)
% P5D_Dubins_RS_vis(g, data, level)
%
% Creates figure to visualize TEB for P5D_Dubins example for the journal 
% paper (example involving 5D car tracking 3D car)

f = figure;
f.Color = 'White';
f.Position = [100 100 800 600];

% Slices to plot
theta_slices = [-15*pi/180; 0];
omega_slice = 0;
as = {'\pi/6'; '0'};

% Formatting
int_font = {'Interpreter', 'LaTeX', 'FontSize', 14};
colors = cell(2,1);
colors{1} = 'm';
colors{2} = 'c';
 L = 0.1;

% Project 4D value function into 2D

data2D = cell(2,1);
for i = 1:length(theta_slices)
  [g2D, data2D{i}] = proj(gX, dataX, [0 0 1 1]);
end

%% Left top and right top: TEB from X and Y
c12 = cell(2,1);
for i = 1:2
  subplot(2,2,i)
  
  c12{i} = visSetIm(g2D, data2D{i}, colors{i}, level);
  c12{i}.LineWidth = 3;
  
  xlim([-L L])
  ylim([-L L])
end

return
subplot(2, 2, 1)

% TEB
c1 = visSetIm(g3D, data3D, 'g', level);
c1.FaceAlpha = 0.8;

% Horizon planes showing slices
hold on
horiz_plane_FaceAlpha = 0.5;
for i = 1:2
  horiz_plane = theta_slices(i)*ones(g2D.N');
  
  surf(g2D.vs{1}, g2D.vs{2}, horiz_plane, 'FaceColor', colors{i}, ...
    'FaceAlpha', horiz_plane_FaceAlpha, 'EdgeColor', 'none');
end

% Formatting
xlim([-L L])
ylim([-L L])
box on
grid on

title("$\mathcal B_{p, \infty}$ (TEB)", int_font{:})
xlabel('$x_r$', int_font{:})
ylabel('$y_r$', int_font{:})
zlabel('$\theta_r$', int_font{:})


%% Right top and Left bottom: Plot value function of 2D slice
for i = 1:2
  subplot(2, 2, i+1)
  
  % Value function
  surf(g2D.vs{1}, g2D.vs{2}, -data2D{i}', 'FaceColor', colors{i}, ...
    'FaceAlpha', 0.8, 'EdgeColor', 'none');
  hold on
  
  horiz_plane = -level*ones(g2D.N');
  surf(g2D.vs{1}, g2D.vs{2}, horiz_plane, 'FaceColor', 'k', 'FaceAlpha', ...
    0.5, 'EdgeColor', 'none');
  
  % TEB
  c23 = visSetIm(g2D, data2D{i}, colors{i}, level);
  c23.LineWidth = 3;
  
  % Formatting
  xlim([-L L])
  ylim([-L L])
  zlim([0 0.1*L])
  box on
  grid on
  
  title_str = ...
    sprintf('$V_\\infty(r)$ projection slice at $\\theta_r = %s$', as{i});
  title(title_str, int_font{:})
  xlabel('$x_r$', int_font{:})
  ylabel('$y_r$', int_font{:})
  zlabel('Value', int_font{:})
end

%% Right bottom: TEB
subplot(2, 2, 4)

% TEB
c4 = cell(2,1);
leg_str = cell(2,1);
for i = 1:2
  c4{i} = visSetIm(g2D, data2D{i}, colors{i}, level);
  c4{i}.LineWidth = 3;
  hold on
  
  leg_str{i} = sprintf('$\\theta_r = %s$', as{i});
end

% Formatting
xlim([-L L])
ylim([-L L])
box on
grid on

title("Slices of $\mathcal B_{p, \infty}$", int_font{:})
xlabel('$x$', int_font{:})
ylabel('$y$', int_font{:})

legend([c4{1} c4{2}], leg_str, int_font{:})
end