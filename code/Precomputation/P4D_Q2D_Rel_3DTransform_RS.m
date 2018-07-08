function [data,tau,sD,teb]=P4D_Q2D_Rel_3DTransform_RS(gN, accuracy,...
    visualize_sylvia, visualize_sumeet)
%Q10D_Q4D_RS Summary of this function goes here
%   Detailed explanation goes here

if nargin < 1
  gN = [51; 51; 51];
end

if nargin < 2
  accuracy = 'high';
end

if nargin <3
    visualize_sylvia = 1;
end

if nargin <4
    visualize_sumeet = 1;
end

%% Grid and cost
gMin = [-.5; -.5; -1];
gMax = [ .5;  .5;  1];
sD.grid = createGrid(gMin, gMax, gN,3);

% select which cost function to use
data0 = sD.grid.xs{1}.^2 + sD.grid.xs{2}.^2;
%data0 =sD.grid.xs{1}.^2 + sD.grid.xs{2}.^2 + sD.grid.xs{3}.^2;

% visualize cost function
[g2D, data02D] = proj(sD.grid,data0,[0 0 1],'max');

if visualize_sylvia
  figure(1)
  clf
  subplot(1,2,1)
  surf(g2D.xs{1}, g2D.xs{2}, sqrt(data02D))
end

%% Dynamical system
wMin = -1;
wMax = 1;

aMin =-1;
aMax = 1;

pMax = .1;

dMax = [0; 0; 0];
dMin = [0; 0; 0];

dims = 1:3;

sD.dynSys = P4D_Q2D_Rel_3DTransform([], wMin, wMax, aMin, aMax, pMax, dMin, dMax,dims);

%% Otherparameters
sD.uMode = 'min';
sD.dMode = 'max';
sD.accuracy = accuracy;


  if visualize_sylvia
    extraArgs.visualize = true;
    extraArgs.RS_level = .1;
    extraArgs.fig_num = 2;
    extraArgs.deleteLastPlot = true;
  end
  
  % bounds on time
  dt = 0.1;
  tMax = 10;
  tau = 0:dt:tMax;
  
  
  extraArgs.stopConverge = true;
  
  % convergence metric
  extraArgs.convergeThreshold = 0.1*dt;
  
  [data, tau] = HJIPDE_solve(data0, tau, sD, 'maxVOverTime', extraArgs);
  
  % get the final value function
  data = data(:,:,:,end);
  
  if visualize_sylvia
    figure(1)
    subplot(1,2,2)
    [g2D, data2D] = proj(sD.grid, data, [0 0 1], 'min');
    data2D = sqrt(data2D);
    surf(g2D.xs{1}, g2D.xs{2}, data2D)
    
       figure(2)
    clf
    alpha = .2;
    levels = [.01, .05, .1];
    
    [g2D, data2D] = proj(sD.grid,data,[0 0 1],1);%'max');
    small = .05;
    subplot(2,3,1)
    h0 = visSetIm(sD.grid, sqrt(data0), 'blue', levels(1)+small);
    h0.FaceAlpha = alpha;
    hold on
    h = visSetIm(sD.grid, sqrt(data), 'red', levels(1));
    axis([-levels(3)-small levels(3)+small ...
      -levels(3)-small levels(3)+small -pi pi])
    axis square
    
    subplot(2,3,4)
    h0 = visSetIm(g2D, sqrt(data02D), 'blue', levels(1)+small);
    hold on
    h = visSetIm(g2D, sqrt(data2D), 'red', levels(1));
    axis([-levels(3)-small levels(3)+small ...
      -levels(3)-small levels(3)+small])
    title(['R = ' num2str(levels(1))])
    axis square
    
    subplot(2,3,2)
    h0 = visSetIm(sD.grid, sqrt(data0), 'blue', levels(2)+small);
    h0.FaceAlpha = alpha;
    hold on
    h = visSetIm(sD.grid, sqrt(data), 'red', levels(2));
    axis([-levels(3)-small levels(3)+small ...
      -levels(3)-small levels(3)+small -pi pi])
    title(['t = ' num2str(tau(end)) ' s'])
    axis square
    
    subplot(2,3,5)
    h0 = visSetIm(g2D, sqrt(data02D), 'blue', levels(2)+small);
    hold on
    h = visSetIm(g2D, sqrt(data2D), 'red', levels(2));
    axis([-levels(3)-small levels(3)+small ...
      -levels(3)-small levels(3)+small])
    title(['R = ' num2str(levels(2))])
    axis square
    
    subplot(2,3,3)
    h0 = visSetIm(sD.grid, sqrt(data0), 'blue', levels(3)+small);
    h0.FaceAlpha = alpha;
    hold on
    h = visSetIm(sD.grid, sqrt(data), 'red', levels(3));
    axis([-levels(3)-small levels(3)+small ...
      -levels(3)-small levels(3)+small -pi pi])
    axis square

    subplot(2,3,6)
    h0 = visSetIm(g2D, sqrt(data02D), 'blue', levels(3)+small);
    hold on
    h = visSetIm(g2D, sqrt(data2D), 'red', levels(3));
    axis([-levels(3)-small levels(3)+small ...
      -levels(3)-small levels(3)+small])
    title(['R = ' num2str(levels(3))])
    axis square
    
    set(gcf,'Color','white')
  end


  
  %tracking error bound
  teb = sqrt(min(data(:)));
  if teb == 0
      keyboard
      teb = .05;
  end
  
  %% Save and output worst value
  %save(sprintf('%s_%f.mat', mfilename, now), 'sD', 'data', 'teb', '-v7.3');
  
  %% Adapt Sumeet's visualization code
  
  if visualize_sumeet
      %Single (v,theta) slice
      figure(5)
      clf
      th = 0;
      v = 0.013;
      
      [g2D, data2D_vslice] = proj(sD.grid, data, [0 0 1], v);
      data2D_vslice(data2D_vslice>teb) = NaN;
      
      %h = surf(g2D.xs{1}, g2D.xs{2}, data2D_vslice);
      contourf(cos(th)*g2D.xs{1}-sin(th)*g2D.xs{2},...
          sin(th)*g2D.xs{1}+cos(th)*g2D.xs{2},...
          data2D_vslice,'linestyle','none'); hold on
      
      xlabel('$e_x''$','interpreter','latex'); ylabel('$e_y''$','interpreter','latex');
      set(findall(gcf,'type','text'),'FontSize',38);set(gca,'FontSize',38)
      set(gcf,'Color','w');
      grid on
      axis equal
      colorbar
      
      % %all (v,theta)
      figure()
      theta = linspace(-pi, pi, 20);
      vel = linspace(-.1, .1, 20);
      
      for k = 1:length(vel)
          v = vel(k);
          [~, data2D_temp] = proj(sD.grid, data, [0 0 1], v);
          data2D_temp(data2D_temp>teb) = NaN;
          for j = 1:length(theta)
              th = theta(j);
              contourf(cos(th)*g2D.xs{1}-sin(th)*g2D.xs{2},...
                  sin(th)*g2D.xs{1}+cos(th)*g2D.xs{2},...
                  data2D_temp,'linestyle','none'); hold on
          end
      end
      grid on
      axis equal
      colorbar
      xlabel('$e_x''$','interpreter','latex'); ylabel('$e_y''$','interpreter','latex');
      set(findall(gcf,'type','text'),'FontSize',38);set(gca,'FontSize',38)
      set(gcf,'Color','w');
  end
end

