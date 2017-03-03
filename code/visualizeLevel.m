function [hCostS, hCostC, hValueS, hValueC, hV1, hL1] = visualizeLevel(g,data,data0,type, cost,valExtraStates,fig)
% inputs:
% g - grid
% data - data that has been computed with min w/ target
% type - quad or plane
% cost - square or circle (depending how you set the initial cost)
% valExtraStates - values at the non-position states

figure(fig)
clf
colormap('winter');


%% Plane
if strcmp(type,'plane')
  
  if strcmp(cost,'square')
    data0 = shapeRectangleByCorners(g,[0 0 -Inf -Inf],[0 0 Inf Inf]);
  elseif strcmp(cost,'circle')
    
    data0 = sqrt(g.xs{1}.^2 + g.xs{2}.^2);
  end
  
  
  %find terminal cost
  
  
  %largest cost on the map
  costMax = g.max(1);%max(data0(:));
  costMin = g.min(1);%min(data0(:));
  
  %project onto valExtraStates
  [gProj, data0Proj] = proj(g, data0, [0 0 1 1], valExtraStates);
  
  %project onto valExtraStates
  data = -data;
  [~, dataProj] = proj(g,data,[0 0 1 1],valExtraStates);
  subplot(1,3,2)
  
  %Find a few good levels to plot for next section
  levelMin = max(min(dataProj(:)), costMin);
  levelMax = min(max(dataProj(:)),costMax);
  levels = linspace(levelMin,levelMax,8);
  levels = levels([2,4,6]);
  
  %Plot
  hValue = surfc(gProj.xs{1},gProj.xs{2},dataProj);
  title(['Value Function, v = ' num2str(valExtraStates(1)) ...
    ' m/s and \theta = ' num2str(valExtraStates(2)) ' rad'],'FontSize',15)
  
  hValue(2).ContourZLevel = levelMin - .05;
  hValue(2).LevelList = hValue(2).LevelList(hValue(2).LevelList <= levelMax);
  
  zlim([levelMin-.05 levelMax]);
  caxis([levelMin levelMax]);
  axis square
  axis([g.min(1) g.max(1) g.min(2) g.max(2) levelMin-.05 levelMax])
  xlabel('$x$','Interpreter','latex','FontSize',20)
  ylabel('$y$','Interpreter','latex','FontSize',20)
  zlabel('$V(z)$','Interpreter','latex','FontSize',20)
  
  %plot cost
  subplot(1,3,1)
  hCost = surfc(gProj.xs{1},gProj.xs{2},data0Proj);
  title(['Cost Function, v = ' num2str(valExtraStates(1)) ...
    ' m/s and \theta = ' num2str(valExtraStates(2)) ' rad'],'FontSize',15);
  hCost(2).LevelList = hCost(2).LevelList(hCost(2).LevelList <= levelMax);
  hCost(2).ContourZLevel = 0; %levelMin - .05;
  
  zlim([0 levelMax]);
  caxis([0 levelMax]);
  axis square
  axis([g.min(1) g.max(1) g.min(2) g.max(2) 0 levelMax])
  xlabel('$x$','Interpreter','latex','FontSize',20)
  ylabel('$y$','Interpreter','latex','FontSize',20)
  zlabel('$l(z)$','Interpreter','latex','FontSize',20)
  
  %get data
  data = -data;
  %data(data>costMax) = nan;
  
  subplot(1,3,3)
  [~,hV1]=contour(gProj.xs{1},gProj.xs{2},dataProj,levels,...
    'Linestyle','--','LineWidth',2);
  hold on
  [~,hL1] = contour(gProj.xs{1},gProj.xs{2},data0Proj,levels,...
    'LineWidth',2);
  axis([g.min(1) g.max(1) g.min(1) g.max(1)])
  title('Mapping initial state to tracking error bound','FontSize',15)
  axis square
  set(gcf,'Color','white')
  colorbar
  xlabel('$x$','Interpreter','latex','FontSize',20)
  ylabel('$y$','Interpreter','latex','FontSize',20)
  
%% Quad
elseif strcmp(type,'quad')
%   if strcmp(cost,'square')
%     data0 = shapeRectangleByCorners(g,[0 -Inf 0 -Inf],[0 Inf 0 Inf]);
%   elseif strcmp(cost,'circle')
%     data0 = sqrt(g.xs{1}.^2 + g.xs{3}.^2);
%   elseif strcmp(cost,'quadratic')
%     data0 = (g.xs{1}.^2 + g.xs{3}.^2);
%     data0=sqrt(data0);
%   elseif strcmp(cost,'quadratic_2x2D')
%     data01 = g.xs{1}.^2;
%     data02 = g.xs{3}.^2;
%     data0 = max(data01,data02);
%     data0 = sqrt(data0);
%   end
  
  %largest cost on the map
  costMax = g.max(1);%max(data0(:));
  costMin = g.min(1);%min(data0(:));
  
  %project cost onto valExtraStates
  [gProj, data0Proj] = proj(g, data0, [0 1 0 1], valExtraStates);
  
  
  %project data onto valExtraStates
  %data = -data; %make positive so contour maps can understand
  [~, dataProj] = proj(g,data,[0 1 0 1],valExtraStates);
  
  %Find a few good levels to plot for next section
  levelMin = max(min(dataProj(:)), costMin);
  levelMax = min(max(dataProj(:)),costMax);
  levels = linspace(levelMin,levelMax,8);
  levels = levels([2,4,6]);
  
  %plot
  subplot(1,3,2)
  hValueS = surf(gProj.xs{1},gProj.xs{2},dataProj);
  hold on
 [~, hValueC] = contour(gProj.xs{1},gProj.xs{2},dataProj,levels,...
    'LineWidth',2);
  title(['Value Function, v_x = ' num2str(valExtraStates(1)) ...
    ' m/s and v_y = ' num2str(valExtraStates(2)) ' m/s'],'FontSize',15)
  
  %hValueC.ContourZLevel = levelMax + .05;
  hValueC.LevelList = hValueC.LevelList(hValueC.LevelList >= levelMin);
  
  zlim([levelMin 0]);%levelMax + .05]);
  caxis([levelMin levelMax]);
  axis square
  xlabel('$x$','Interpreter','latex','FontSize',20)
  ylabel('$y$','Interpreter','latex','FontSize',20)
  zlabel('$V(s)$','Interpreter','latex','FontSize',20)
  
  %plot cost
  subplot(1,3,1)
  hCostS = surf(gProj.xs{1},gProj.xs{2},data0Proj);
  title(['Cost Function, v_x = ' num2str(valExtraStates(1)) ...
    ' m/s and v_y = ' num2str(valExtraStates(2)) ' m/s'],'FontSize',15);
  hold on
  [~, hCostC] = contour(gProj.xs{1},gProj.xs{2},data0Proj,levels,...
    'LineWidth',2);
  
  hCostC.LevelList = hCostC.LevelList(hCostC.LevelList >= levelMin);
  
  zlim([levelMin 0]);
  caxis([levelMin 0]);
  axis square
  xlabel('$x$','Interpreter','latex','FontSize',20)
  ylabel('$y$','Interpreter','latex','FontSize',20)
  zlabel('$l(s)$','Interpreter','latex','FontSize',20)
  
  subplot(1,3,3)
  [~,hV1]=contour(gProj.xs{1},gProj.xs{2},dataProj,levels,...
    'Linestyle','--','LineWidth',2);
  hold on
  [~,hL1] = contour(gProj.xs{1},gProj.xs{2},data0Proj,levels,...
    'LineWidth',2);
  axis([g.min(1) g.max(1) g.min(1) g.max(1)])
  title('Mapping initial state to tracking error bound','FontSize',15)
  axis square
  set(gcf,'Color','white')
  colorbar
  xlabel('$x$','Interpreter','latex','FontSize',20)
  ylabel('$y$','Interpreter','latex','FontSize',20)

%% Quad10D
elseif strcmp(type,'quad10D')
  
% Reconstruct
vfs.gs = g;
vfs.datas = data;
vfs.tau = 0;
vfs.dims{1} = 1:4;
vfs.dims{2} = 5:8;
vfs.dims{3} = 9:10;

range_lower_X = [vfs.gs{1}.min(1) vfs.gs{1}.min(2) valExtraStates(3)-.1 ...
  valExtraStates(4)-.1];
range_lower_Y = [vfs.gs{2}.min(1) vfs.gs{2}.min(2) valExtraStates(7)-.1 ...
  valExtraStates(8)-.1];
range_lower_Z = [vfs.gs{3}.min(1) vfs.gs{3}.min(2)];
range_lower = [range_lower_X range_lower_Y range_lower_Z];

range_upper_X = [vfs.gs{1}.max(1) vfs.gs{1}.max(2) valExtraStates(3)+.1 ...
  valExtraStates(4)+.1];
range_upper_Y = [vfs.gs{2}.max(1) vfs.gs{2}.max(2) valExtraStates(7)+.1 ...
  valExtraStates(8)+.1];
range_upper_Z = [vfs.gs{3}.max(1) vfs.gs{3}.max(2)];
range_upper = [range_upper_X range_upper_Y range_upper_Z];

vf = reconSC(vfs, range_lower, range_upper, 0, 'min');

vfs0 = vfs;
vfs0.data = data0;
vf0 = reconSC(vfs0, range_lower, range_upper, 0, 'min');
g = vf.g;

if strcmp(cost,'quadratic_decomp')
  data = sqrt(-vf.data);
  data0 = sqrt(-vf0.data);
else
  error 'what cost?'
end

    %largest cost on the map
  costMax = g.max(1);%max(data0(:));
  costMin = g.min(1);%min(data0(:));
  
  if g.dim == 6
  %project cost onto valExtraStates
  [gProj, data0Proj] = proj(g, data0, [0 1 0 1 0 1], ...
    [valExtraStates(2) valExtraStates(6) valExtraStates(10)]);
  %project data onto valExtraStates
  [~, dataProj] = proj(g,data,[0 1 0 1 0 1],...
    [valExtraStates(2) valExtraStates(6) valExtraStates(10)]);
  elseif g.dim == 10
    error 'too many dims!'
  end
  


  
  %Find a few good levels to plot for next section
  levelMin = max(min(dataProj(:)), costMin);
  levelMax = min(max(dataProj(:)),costMax);
  levels = linspace(levelMin,levelMax,8);
  levels = levels([2,4,6]);
  
  %plot
  subplot(1,3,2)
  [gProj2D, dataProj2D] = proj(gProj, dataProj, [0 0 1], 0);
  hValue = surfc(gProj2D.xs{1},gProj2D.xs{2},dataProj2D);
  title(['Value Function, v_x = ' num2str(valExtraStates(1)) ...
    ' m/s, v_y = ' num2str(valExtraStates(2)) ' m/s' ...
    'v_z = ' num2str(valExtraStates(2)) ' m/s'],'FontSize',15)
  
  hValue(2).ContourZLevel = levelMin - .05;
  hValue(2).LevelList = hValue(2).LevelList(hValue(2).LevelList <= levelMax);
  
  zlim([levelMin-.05 levelMax]);
  caxis([levelMin levelMax]);
  axis square
  xlabel('$x$','Interpreter','latex','FontSize',20)
  ylabel('$y$','Interpreter','latex','FontSize',20)
  zlabel('$V(z)$','Interpreter','latex','FontSize',20)
  
  %   %plot cost
  [~, data0Proj2D] = proj(gProj, data0Proj, [0 0 1], 0);
  
  subplot(1,3,1)
  hCost = surfc(gProj2D.xs{1},gProj2D.xs{2},data0Proj2D);
  title(['Cost Function, v_x = ' num2str(valExtraStates(1)) ...
    ' m/s and v_y = ' num2str(valExtraStates(2)) ' m/s'],'FontSize',15);
  
  hCost(2).LevelList = hCost(2).LevelList(hCost(2).LevelList <= levelMax);
  
  zlim([0 levelMax]);
  caxis([0 levelMax]);
  axis square
  xlabel('$x$','Interpreter','latex','FontSize',20)
  ylabel('$y$','Interpreter','latex','FontSize',20)
  zlabel('$l(z)$','Interpreter','latex','FontSize',20)
  
  subplot(1,3,3)
  
  
  [ mesh_xs, mesh_data ] = gridnd2mesh(gProj, dataProj);
  hV1 = patch(isosurface(mesh_xs{:}, mesh_data, levels(3)));
  isonormals(mesh_xs{:}, mesh_data, hV1);
  hV1.FaceColor = 'r';
  hV1.EdgeColor = 'none';
  hV1.FaceAlpha = .4;
  lighting phong
  camlight left
  camlight right
  view(3)
  hold on

    [ mesh_xs0, mesh_data0 ] = gridnd2mesh(gProj, data0Proj);
  hL1 = patch(isosurface(mesh_xs0{:}, mesh_data0, levels(3)));
  isonormals(mesh_xs0{:}, mesh_data0, hL1);
  hL1.FaceColor = 'b';
  hL1.EdgeColor = 'none';
  hL1.FaceAlpha = 1;
  lighting phong
  camlight left
  camlight right
  view(3)
  % [~,hV1]=contour(gProj.xs{1},gProj.xs{2},dataProj,levels,...
  %   'Linestyle','--','LineWidth',2);
  % hold on
  % [~,hL1] = contour(gProj.xs{1},gProj.xs{2},data0Proj,levels,...
  %   'LineWidth',2);
  % axis([g.min(1) g.max(1) g.min(1) g.max(1)])
  % title('Mapping initial state to tracking error bound','FontSize',15)
  % axis square
  % set(gcf,'Color','white')
  % colorbar
  % xlabel('$x$','Interpreter','latex','FontSize',20)
  % ylabel('$y$','Interpreter','latex','FontSize',20)
end


end