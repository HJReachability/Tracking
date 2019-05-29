function [data,tau,sD,teb]=P3D_Q2D_RS(gN, visualize, video)
%Q10D_Q4D_RS Summary of this function goes here
%   Detailed explanation goes here

if nargin < 1
  gN = [111; 111; 51];
end

if nargin < 2
  visualize = 1;
end

if nargin < 3
  video = 1;
end

if video
  %This file makes a gif and an avi file from 3D projections across different
  %numbers of psi and v sections. Right now it's set up to do an xyv slice
  
  gif_out_filename1 = './visualize_tracking_sweep.gif'; %v_goal.gif';   %Setup filename for gif
  avi_out_filename1 = './visualize_tracking_sweep.avi'; %v_goal.avi';
  
  % Have to prepare the video writer object first
  v1 = VideoWriter(avi_out_filename1);
  v1.FrameRate = 30;   %Set framerate of playback. 30 is normal.
  open(v1)   %Opens the file for writing. Make sure to close at the end!!
  
  gif_out_filename2 = './visualize_tracking_function_1.gif'; %v_goal.gif';   %Setup filename for gif
  avi_out_filename2 = './visualize_tracking_function_1.avi'; %v_goal.avi';
  
  v2 = VideoWriter(avi_out_filename2);
  v2.FrameRate = 30;   %Set framerate of playback. 30 is normal.
  open(v2)   %Opens the file for writing. Make sure to close at the end!!
  
    gif_out_filename3 = './visualize_tracking_function_2.gif'; %v_goal.gif';   %Setup filename for gif
  avi_out_filename3 = './visualize_tracking_function_2.avi'; %v_goal.avi';
  
  v3 = VideoWriter(avi_out_filename3);
  v3.FrameRate = 30;   %Set framerate of playback. 30 is normal.
  open(v3)   %Opens the file for writing. Make sure to close at the end!!
end


%% Grid and cost
gMin = [-5; -5; -5];
gMax = [ 5;  5;  5];
sD.grid = createGrid(gMin, gMax, gN,3);

data0 = sD.grid.xs{1}.^2 + sD.grid.xs{2}.^2;
%data0 =sD.grid.xs{1}.^2 + sD.grid.xs{2}.^2 + sD.grid.xs{3}.^2;

[g2D, data02D] = proj(sD.grid,data0,[0 0 1],'max');

if visualize
  figure(1)
  clf
  subplot(1,2,1)
  surf(g2D.xs{1}, g2D.xs{2}, sqrt(data02D))
end

%% Dynamical system
uMin = [-1, -1];
uMax = [1, 1];
%wMin = [-1];
%wMax = [1];

%aMin =[-1];
%aMax = [1];

pMax = [.1, .1];
pMin = [-.1, -.1];

dMax = [0; 0];
dMin = [0; 0];

dims = 1:3;

%P3D_Q2D_Rel(x, uMin, uMax, pMin, pMax, dMin, dMax, v, dims)
sD.dynSys = P3D_Q2D_Rel([], uMin, uMax, pMin, pMax, dMin, dMax);

%% Otherparameters
sD.uMode = 'min';
sD.dMode = 'max';
sD.accuracy = 'low';

if video
  dt = 0.01;
  tMax = 1;
  converge = 0;
  i = 0;
  for i = 1:tMax/dt
      tMaxTemp = dt*i;
      tau = 0:dt:tMaxTemp;
      % tau = 0:dt:dt;
      
      extraArgs.stopConverge = true;
    extraArgs.convergeThreshold = 0.5*dt;
    extraArgs.keepLast = 1;
    
    if i == 0
        data = data0;
        i = 1;
        tNow = 0;
    else
    [data, tau] = HJIPDE_solve(data0, tau, sD, 'maxVWithV0', extraArgs);
    %tNow = tNow + tau(end);
    tNow = tau(end);
    % max over time
    data = max(data,[],4);
%     change = max(abs(data(:)-dataprev(:)));
%     if change <= extraArgs.convergeThreshold
%         converge = 1;
%     end
%     
    end
    [g2D, data2D] = proj(sD.grid,data,[0 0 1],0);%'max');
    levels = [.5, .75, 1];
    levelColor = {[0, .6, .6], [.7 0 .7], [.7 .3 .3]};
    
    figure(1)
    clf
    hV = surf(g2D.xs{1}, g2D.xs{2}, sqrt(data2D));
    hV.LineStyle = 'none';
    hV.FaceLighting = 'phong';
    axis([-1.5 1.5 -1.5 1.5 0 1.5])
    set(gcf, 'Color','white')
    hV.FaceColor = [.7 .7 .7];
    hV.FaceAlpha = .7;
    c = camlight;
    c.Position = [-10 -80 -20];
    axis square
    hold on
    view(40,20)
    title(['T = ' num2str(tNow,'%4.2f') ' s'],'Interpreter','latex','FontSize',20)
    xlabel('$x_r$','Interpreter','latex','FontSize',20)
    ylabel('$y_r$','Interpreter','latex','FontSize',20)
    zlabel('$V(r,T)$','Interpreter','latex','FontSize',20)

    if i == 0   %Make sure this is the loop index == 1      
      frame = getframe(gcf);   %Get data from figue 1
      image_data = frame2im(frame);   %convert data to image information (this goes straight into avi)
      [imind,cm] = rgb2ind(image_data,256);   %convert image information to index and colormap for gif (don't
      
      imwrite(imind,cm,gif_out_filename2,'gif', 'Loopcount',inf);   %If you're on the first pass, make a new gif
      writeVideo(v2,image_data)
    end

      

      frame = getframe(gcf);   %Get data from figue 1
      image_data = frame2im(frame);   %convert data to image information (this goes straight into avi)
      [imind,cm] = rgb2ind(image_data,256);   %convert image information to index and colormap for gif (don't
      
      imwrite(imind,cm,gif_out_filename2,'gif','WriteMode','append');   %If it's not the first pass, append
    writeVideo(v2,image_data)
      h3 = visSetIm(g2D, sqrt(data2D), levelColor{3}, levels(3));
      h3.LineWidth = 3;
      h3.ContourZLevel = levels(3);
        
    h2 = visSetIm(g2D, sqrt(data2D), levelColor{2}, levels(2));
    h2.LineWidth = 3;
    h2.ContourZLevel = levels(2);
    h1 = visSetIm(g2D, sqrt(data2D), levelColor{1}, levels(1));
    h1.LineWidth = 3;
    h1.ContourZLevel = levels(1);
    
        frame = getframe(gcf);   %Get data from figue 1
    image_data = frame2im(frame);   %convert data to image information (this goes straight into avi)
    [imind,cm] = rgb2ind(image_data,256);   %convert image information to index and colormap for gif (don't
    
    if i == 0   %Make sure this is the loop index == 1
      imwrite(imind,cm,gif_out_filename3,'gif', 'Loopcount',inf);   %If you're on the first pass, make a new gif
    else
      imwrite(imind,cm,gif_out_filename3,'gif','WriteMode','append');   %If it's not the first pass, append
    end
    writeVideo(v3,image_data)

    figure(2)
    clf
    alpha = .2;
    
    small = .01;
    subplot(2,3,1)
    h0 = visSetIm(sD.grid, sqrt(data0), 'blue', levels(1)+small);
    h0.FaceAlpha = alpha;
    hold on
    h = visSetIm(sD.grid, sqrt(data), levelColor{1}, levels(1));
    axis([-levels(3)-small levels(3)+small ...
      -levels(3)-small levels(3)+small -pi pi])
    xlabel('$x_r$','Interpreter','latex','FontSize',20)
    ylabel('$y_r$','Interpreter','latex','FontSize',20)
    zlabel('$\theta$','Interpreter','latex','FontSize',20)
    axis square
    
    subplot(2,3,4)
    h0 = visSetIm(g2D, sqrt(data02D), 'blue', levels(1));
    h0.LineWidth = 2;
    hold on
    h = visSetIm(g2D, sqrt(data2D), levelColor{1}, levels(1));
    axis([-levels(3)-small levels(3)+small ...
      -levels(3)-small levels(3)+small])
    title(['V(r,T) = ' num2str(levels(1))],'Interpreter','latex','FontSize',20)
    xlabel('$x_r$','Interpreter','latex','FontSize',20)
    ylabel('$y_r$','Interpreter','latex','FontSize',20)
    h.LineWidth = 2;
    axis square
    
    subplot(2,3,2)
    h0 = visSetIm(sD.grid, sqrt(data0), 'blue', levels(2)+small);
    h0.FaceAlpha = alpha;
    hold on
    h = visSetIm(sD.grid, sqrt(data), levelColor{2}, levels(2));
    axis([-levels(3)-small levels(3)+small ...
      -levels(3)-small levels(3)+small -pi pi])
    title(['T = ' num2str(tNow,'%4.2f') ' s'],'Interpreter','latex','FontSize',20)
    xlabel('$x_r$','Interpreter','latex','FontSize',20)
    ylabel('$y_r$','Interpreter','latex','FontSize',20)
    zlabel('$\theta$','Interpreter','latex','FontSize',20)
    axis square
    
    subplot(2,3,5)
    h0 = visSetIm(g2D, sqrt(data02D), 'blue', levels(2));
    h0.LineWidth = 2;
    hold on
    h = visSetIm(g2D, sqrt(data2D), levelColor{2}, levels(2));
    axis([-levels(3)-small levels(3)+small ...
      -levels(3)-small levels(3)+small])
    h.LineWidth = 2;
    title(['V(r,T) = ' num2str(levels(2))],'Interpreter','latex','FontSize',20)
    xlabel('$x_r$','Interpreter','latex','FontSize',20)
    ylabel('$y_r$','Interpreter','latex','FontSize',20)
    axis square
    
    subplot(2,3,3)
    h0 = visSetIm(sD.grid, sqrt(data0), 'blue', levels(3)+small);
    h0.FaceAlpha = alpha;
    hold on
    h = visSetIm(sD.grid, sqrt(data), levelColor{3}, levels(3));
    axis([-levels(3)-small levels(3)+small ...
      -levels(3)-small levels(3)+small -pi pi])
    xlabel('$x_r$','Interpreter','latex','FontSize',20)
    ylabel('$y_r$','Interpreter','latex','FontSize',20)
    zlabel('$\theta$','Interpreter','latex','FontSize',20)
    axis square

    subplot(2,3,6)
    h0 = visSetIm(g2D, sqrt(data02D), 'blue', levels(3));
    h0.LineWidth = 2;
    hold on
    h = visSetIm(g2D, sqrt(data2D), levelColor{3}, levels(3));
    axis([-levels(3)-small levels(3)+small ...
      -levels(3)-small levels(3)+small])
    title(['V(r,T) = ' num2str(levels(3))],'Interpreter','latex','FontSize',20)
    xlabel('$x_r$','Interpreter','latex','FontSize',20)
    ylabel('$y_r$','Interpreter','latex','FontSize',20)
    h.LineWidth = 2;
    axis square
    
    set(gcf,'Color','white')
    frame = getframe(gcf);   %Get data from figue 1
    image_data = frame2im(frame);   %convert data to image information (this goes straight into avi)
    [imind,cm] = rgb2ind(image_data,256);   %convert image information to index and colormap for gif (don't
    
    if i == 0   %Make sure this is the loop index == 1
      imwrite(imind,cm,gif_out_filename1,'gif', 'Loopcount',inf);   %If you're on the first pass, make a new gif
    else
      imwrite(imind,cm,gif_out_filename1,'gif','WriteMode','append');   %If it's not the first pass, append
    end
    writeVideo(v1,image_data)
    %dataprev = data;
   end
  close(v1)
   close(v2)
 close(v3)
else
  if visualize
    extraArgs.visualize.valueFunction = true;
    extraArgs.visualize.sliceLevel = 2;
    extraArgs.visualize.figNum = 2;
    extraArgs.visualize.plotData.plotDims = [1 1 0];
    extraArgs.visualize.plotData.projpt = 0;
    extraArgs.visualize.deleteLastPlot = true;
  end
  dt = 0.1;
  tMax = .75;
  tau = 0:dt:tMax;
  
  extraArgs.keepLast = 1;
  extraArgs.stopConverge = true;
  extraArgs.convergeThreshold = dt;%0.5*dt;
  
  tic
  [data, tau] = HJIPDE_solve(data0, tau, sD, 'maxVWithV0', extraArgs);
  runtime = toc;
  
  %data = max(data,[],4);
  if visualize
    figure(1)
    subplot(1,2,2)
    [g2D, data2D] = proj(sD.grid, data, [0 0 1], 'min');
    data2D = sqrt(data2D);
    surf(g2D.xs{1}, g2D.xs{2}, data2D)
    
       figure(3)
    clf
    alpha = .2;
    levels = [.5, .75, 1];
    
    [g2D, data2D] = proj(sD.grid,data,[0 0 1],0);%'max');
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
    title(['t = ' num2str(tNow,'%4.2f') ' s'])
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
end



%tracking error bound
teb = sqrt(min(data(:)))
%% Save and output worst value
%save(sprintf('%s_%f.mat', mfilename, now), 'sD', 'data', '-v7.3');

% for i = 1:size(data,5)
%   data_i = data(:,:,:,:,i);
%   max(data_i(:))
% end

%keyboard
end

