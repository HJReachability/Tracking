function [data,tau,sD,teb_level]=P4D_Q2D_Rel_3DTransform_RS(gN, accuracy,...
    visualize_sylvia, visualize_sumeet)
%Q10D_Q4D_RS Summary of this function goes here
%   Detailed explanation goes here

if nargin < 1
  gN = [201; 201; 201];
end

if nargin < 2
  accuracy = 'veryHigh';
end

if nargin <3
    visualize_sylvia = 1;
end

if nargin <4
    visualize_sumeet = 1;
end

buffer = .005;

%% Grid and cost
gMin = [-1; -1; -1];
gMax = [ 1;  1;  1];
sD.grid = createGrid(gMin, gMax, gN);

% select which cost function to use
%data0 = sD.grid.xs{1}.^2 + sD.grid.xs{2}.^2;
data0 =sD.grid.xs{1}.^2 + sD.grid.xs{2}.^2 + sD.grid.xs{3}.^2;

% visualize cost function
[g2D, data02D] = proj(sD.grid,data0,[0 0 1],'max');

if visualize_sylvia
  figure(1)
  clf
  subplot(1,2,1)
  surf(g2D.xs{1}, g2D.xs{2}, data02D)
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
    extraArgs.RS_level = .01;
    extraArgs.fig_num = 2;
    extraArgs.deleteLastPlot = true;
  end
  
  % bounds on time
  dt = 0.1;
  tMax = 10;
  tau = 0:dt:tMax;
  
  
  extraArgs.stopConverge = true;
  
  % convergence metric
  extraArgs.convergeThreshold = 0.05*dt;
  
  [data, tau] = HJIPDE_solve(data0, tau, sD, 'maxVOverTime', extraArgs);
  
  % get the final value function
  data = data(:,:,:,end);
  
    %tracking error bound
  teb_level = min(data(:)) + buffer;
  if (teb_level - buffer) == 0
      keyboard
      %teb = .05;
  end
  
  
  if visualize_sylvia
    figure(1)
    subplot(1,2,2)
    v = 0.013;
    [g2D, data2D] = proj(sD.grid, data, [0 0 1], v);
    %data2D = sqrt(data2D);
    surf(g2D.xs{1}, g2D.xs{2}, data2D)
    set(gca,'FontSize',20)
    
       figure(3)
    clf
    alpha = .2;
    levels = [teb_level-buffer/2, teb_level, teb_level+buffer/2];
    axisbounds = [gMin(1) gMax(1) gMin(2) gMax(2) gMin(3) gMax(3)];
    xlabel('$e_x''$','interpreter','latex')
    ylabel('$e_y''$','interpreter','latex');
    set(gca,'FontSize',20)
    
    [g2D, data2D] = proj(sD.grid,data,[0 0 1],1);%'max');
    small = .05;
    subplot(2,3,1)
    h0 = visSetIm(sD.grid, data0, 'blue', levels(1)+small);
    h0.FaceAlpha = alpha;
    hold on
    h = visSetIm(sD.grid, data, 'red', levels(1));
    axis(axisbounds)
    axis square
    xlabel('$e_x''$','interpreter','latex')
    ylabel('$e_y''$','interpreter','latex');
    zlabel('$v$','interpreter','latex');
    set(gca,'FontSize',20)
    
    subplot(2,3,4)
    h0 = visSetIm(g2D, data02D, 'blue', levels(1)+small);
    hold on
    h = visSetIm(g2D, data2D, 'red', levels(1));
    axis(axisbounds)
    title(['Value Level = ' num2str(levels(1),1)])
    axis square
    xlabel('$e_x''$','interpreter','latex')
    ylabel('$e_y''$','interpreter','latex');
    set(gca,'FontSize',20)
    
    subplot(2,3,2)
    h0 = visSetIm(sD.grid, data0, 'blue', levels(2)+small);
    h0.FaceAlpha = alpha;
    hold on
    h = visSetIm(sD.grid, data, 'red', levels(2));
    axis(axisbounds)
    title(['t = ' num2str(tau(end)) ' s'])
    axis square
    xlabel('$e_x''$','interpreter','latex')
    ylabel('$e_y''$','interpreter','latex');
    zlabel('$v$','interpreter','latex');
    set(gca,'FontSize',20)
    
    subplot(2,3,5)
    h0 = visSetIm(g2D, data02D, 'blue', levels(2)+small);
    hold on
    h = visSetIm(g2D, data2D, 'red', levels(2));
    axis(axisbounds)
    title(['Value Level = ' num2str(levels(2),1)])
    axis square
    xlabel('$e_x''$','interpreter','latex')
    ylabel('$e_y''$','interpreter','latex');
    set(gca,'FontSize',20)
    
    subplot(2,3,3)
    h0 = visSetIm(sD.grid, data0, 'blue', levels(3)+small);
    h0.FaceAlpha = alpha;
    hold on
    h = visSetIm(sD.grid, data, 'red', levels(3));
    axis(axisbounds)
    axis square
    xlabel('$e_x''$','interpreter','latex')
    ylabel('$e_y''$','interpreter','latex');
    zlabel('$v$','interpreter','latex');
    set(gca,'FontSize',20)

    subplot(2,3,6)
    h0 = visSetIm(g2D, data02D, 'blue', levels(3)+small);
    hold on
    h = visSetIm(g2D, data2D, 'red', levels(3));
    axis(axisbounds)
    title(['Value Level = ' num2str(levels(3),1)])
    axis square
    xlabel('$e_x''$','interpreter','latex')
    ylabel('$e_y''$','interpreter','latex');
    
    set(gcf,'Color','white')
    set(findall(gcf,'type','text'),'FontSize',20)
  end

%savefig(['plots_3D_gN_' num2str(gN(1)) '.fig'])
%export_fig HJ_TEB_3Dplots.pdf

    
  %% Adapt Sumeet's visualization code
  
  if visualize_sumeet
      %Single (v,theta) slice
      figure(4)
      clf
      th = 0;
      v = 0.013;
      axisbounds = [-.2 .2 -.2 .2];
      
      
      cmap = cbrewer('seq','YlGnBu',64);
      colormap(cmap);
      
      [g2D, data2D_vslice] = proj(sD.grid, data, [0 0 1], v);
      data2D_vslice(data2D_vslice>teb_level) = NaN;
      
      %h = surf(g2D.xs{1}, g2D.xs{2}, data2D_vslice);
      contourf(cos(th)*g2D.xs{1}-sin(th)*g2D.xs{2},...
          sin(th)*g2D.xs{1}+cos(th)*g2D.xs{2},...
          data2D_vslice,'linestyle','none'); hold on
      
      xlabel('$e_x''$','interpreter','latex'); ylabel('$e_y''$','interpreter','latex');
      set(findall(gcf,'type','text'),'FontSize',38);set(gca,'FontSize',38)
      set(gcf,'Color','w');
      grid on
      axis equal
      axis(axisbounds)
      c1 = colorbar;
      c1.Position =[0.7541 0.2343 0.0265 0.6002];
      
      
      % %all (v,theta)
      figure(5)
      clf
      theta = linspace(-pi, pi, 20);
      vel = linspace(-.5, .5, 20);
      
      for k = 1:length(vel)
          v = vel(k);
          [~, data2D_temp] = proj(sD.grid, data, [0 0 1], v);
          data2D_temp(data2D_temp>teb_level) = NaN;
          for j = 1:length(theta)
              th = theta(j);
              contourf(cos(th)*g2D.xs{1}-sin(th)*g2D.xs{2},...
                  sin(th)*g2D.xs{1}+cos(th)*g2D.xs{2},...
                  data2D_temp,'linestyle','none'); hold on
          end
      end
      grid on
      axis equal
      axis(axisbounds)
      c2 = colorbar;
      c2.Position =[0.7541 0.2343 0.0265 0.6002];
      xlabel('$e_x''$','interpreter','latex'); ylabel('$e_y''$','interpreter','latex');
      set(findall(gcf,'type','text'),'FontSize',38);set(gca,'FontSize',38)
      set(gcf,'Color','w');
  end
end

