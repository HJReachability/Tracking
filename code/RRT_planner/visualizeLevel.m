function [hCostS, hCostC, hValueS, hValueC, hV, hL] = visualizeLevel(g,data,data0,type, cost,valExtraStates,fig)
% inputs:
% g - grid
% data - data that has been computed with min w/ target
% type - quad or plane
% cost - square or circle (depending how you set the initial cost)
% valExtraStates - values at the non-position states

figure(fig)
clf
colormap('winter');
converge = 0;

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
  title(['Reward Function, v = ' num2str(valExtraStates(1)) ...
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
  
  subplot(2,3,2)
  [~,hV]=contour(gProj.xs{1},gProj.xs{2},dataProj,levels,...
    'Linestyle','--','LineWidth',2);
  hold on
  [~,hL] = contour(gProj.xs{1},gProj.xs{2},data0Proj,levels,...
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
  data0Proj = -data0Proj;
  
  %project data onto valExtraStates
  %data = -data; %make positive so contour maps can understand
  data = sqrt(-data);
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
  title('Value Function')
  %title(['Value Function, v_x = ' num2str(valExtraStates(1)) ...
  %  ' m/s and v_y = ' num2str(valExtraStates(2)) ' m/s'],'FontSize',15)
  
  %hValueC.ContourZLevel = levelMax + .05;
  hValueC.LevelList = hValueC.LevelList(hValueC.LevelList >= levelMin);
  
  zlim([0 levelMax]);%levelMax + .05]);
  caxis([levelMin levelMax]);
  axis square
  xlabel('$x$','Interpreter','latex','FontSize',20)
  ylabel('$y$','Interpreter','latex','FontSize',20)
  zlabel('$V(s_r)$','Interpreter','latex','FontSize',20)
  
  %plot cost
  subplot(1,3,1)
  hCostS = surf(gProj.xs{1},gProj.xs{2},data0Proj);
  title('Cost Function')
  %title(['Cost Function, v_x = ' num2str(valExtraStates(1)) ...
  %  ' m/s and v_y = ' num2str(valExtraStates(2)) ' m/s'],'FontSize',15);
  hold on
  [~, hCostC] = contour(gProj.xs{1},gProj.xs{2},data0Proj,levels,...
    'LineWidth',2);
  
  hCostC.LevelList = hCostC.LevelList(hCostC.LevelList >= levelMin);
  
  zlim([0 levelMax]);
  caxis([0 levelMax]);
  axis square
  xlabel('$x$','Interpreter','latex','FontSize',20)
  ylabel('$y$','Interpreter','latex','FontSize',20)
  zlabel('$l(s_r)$','Interpreter','latex','FontSize',20)
  
  subplot(1,3,3)
  [~,hV]=contour(gProj.xs{1},gProj.xs{2},dataProj,levels,...
    'Linestyle','--','LineWidth',2);
  hold on
  [~,hL] = contour(gProj.xs{1},gProj.xs{2},data0Proj,levels,...
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
  textSize = 20;
  titleTextSize = 15;
  cmap = colormap('winter');
% Reconstruct
vfs.gs = g;
vfs.datas = data;
vfs.tau = 0;
vfs.dims{1} = 1:4;
vfs.dims{2} = 5:8;
vfs.dims{3} = 9:10;

small = 1e-3;
range_lower = valExtraStates - 0.5*[vfs.gs{1}.dx; vfs.gs{2}.dx; vfs.gs{3}.dx];
range_upper = valExtraStates + 0.5*[vfs.gs{1}.dx; vfs.gs{2}.dx; vfs.gs{3}.dx];

range_lower([1 5 9]) = [vfs.gs{1}.min(1); vfs.gs{2}.min(1); vfs.gs{3}.min(1)]-small;
range_upper([1 5 9]) = [vfs.gs{1}.max(1); vfs.gs{2}.max(1); vfs.gs{3}.max(1)]+small;

vf = reconSC(vfs, range_lower, range_upper, 0, 'min');

if converge
  [gConv{1},dataCon{1}]=proj(vfs.gs{1},vfs.datas{1},[0 1 1 1],'max');
  [gConv{2},dataCon{2}]=proj(vfs.gs{2},vfs.datas{2},[0 1 1 1],'max');
  [gConv{3},dataCon{3}]=proj(vfs.gs{3},vfs.datas{3},[0 1],'max');
  vfs0.gs = gConv;
  vfs0.datas = dataCon;
  vfs0.dims={1;2;3};
  vfs0.tau=0;
  range_lower = [vfs0.gs{1}.min; vfs0.gs{2}.min; vfs0.gs{3}.min]-small;
  range_upper = [vfs0.gs{1}.max; vfs0.gs{2}.max; vfs0.gs{3}.max]+small;
  vf0 = reconSC(vfs0, range_lower, range_upper, 0, 'min');
else
  vfs0 = vfs;
  vfs0.datas = data0;
  vf0 = reconSC(vfs0, range_lower, range_upper, 0, 'min');  
end

gProj = vf.g;

if strcmp(cost,'quadratic_decomp')
  dataProj = sqrt(-vf.data);
  data0Proj = sqrt(-vf0.data);
elseif strcmp(cost, 'oneNorm')
  dataProj = -vf.data;
  data0Proj = -vf0.data;
else
  error 'what cost?'
end

    %largest cost on the map
  costMax = gProj.max(1);%max(data0(:));
  costMin = gProj.min(1);%min(data0(:));
  
  if gProj.dim == 6
  %project cost onto valExtraStates
  [gProj, data0Proj] = proj(g, data0, [0 1 0 1 0 1], ...
    [valExtraStates(2) valExtraStates(6) valExtraStates(10)]);
  %project data onto valExtraStates
  [~, dataProj] = proj(g,data,[0 1 0 1 0 1],...
    [valExtraStates(2) valExtraStates(6) valExtraStates(10)]);
  elseif gProj.dim == 10
    error 'too many dims!'
  end
  
  
  %Find a few good levels to plot for next section
  levelMin = max(min(dataProj(:)), costMin);
  levelMax = min(max(dataProj(:)),costMax);
  levels = linspace(levelMin,levelMax,100);
  levels = levels([5, 20, 40]);
  
  %plot
  subplot(2,3,4)
  [gProj2D, dataProj2D] = proj(gProj, dataProj, [0 0 1], 0);
  hValueS = surf(gProj2D.xs{1},gProj2D.xs{2},dataProj2D);
  hold on
  [~, hValueC] = contour(gProj2D.xs{1},gProj2D.xs{2},dataProj2D,levels,...
    'LineWidth',2);
  set(gca,'FontSize',textSize)
  title('Value Function','FontSize',titleTextSize)
  %title(['Value Function, v_x = ' num2str(valExtraStates(1)) ...
  %  ' m/s, v_y = ' num2str(valExtraStates(2)) ' m/s' ...
  %  'v_z = ' num2str(valExtraStates(2)) ' m/s'],'FontSize',15)
  
  hValueC.ContourZLevel = levelMin - .05;
  hValueC.LevelList = hValueC.LevelList(hValueC.LevelList <= levelMax);
  
  zlim([0 levelMax + .05]);
  caxis([levelMin levelMax]);
  axis square
  xlabel('$x$','Interpreter','latex','FontSize',textSize)
  ylabel('$y$','Interpreter','latex','FontSize',textSize)
  zlabel('$V(s)$','Interpreter','latex','FontSize',textSize)
  
  %   %plot cost
  [~, data0Proj2D] = proj(gProj, data0Proj, [0 0 1], 0);
  
  subplot(2,3,1)
  hCostS = surf(gProj2D.xs{1},gProj2D.xs{2},data0Proj2D);
  hold on
  [~, hCostC] = contour(gProj2D.xs{1},gProj2D.xs{2},data0Proj2D,levels,...
    'LineWidth',2);
  set(gca,'FontSize',textSize)
  title('Reward Function','FontSize',titleTextSize)
  %title(['Cost Function, v_x = ' num2str(valExtraStates(1)) ...
  %  ' m/s and v_y = ' num2str(valExtraStates(2)) ' m/s'],'FontSize',15);
  
  hCostC.LevelList = hCostC.LevelList(hCostC.LevelList >= levelMin);
  
  zlim([0 levelMax]);
  caxis([0 levelMax]);
  axis square
  xlabel('$x$','Interpreter','latex','FontSize',textSize)
  ylabel('$y$','Interpreter','latex','FontSize',textSize)
  zlabel('$l(s)$','Interpreter','latex','FontSize',textSize)
  colorbar
  
  for i = 1:length(levels)
  color = cmap(10*i,:);
  
  if i <= 2
  subplot(2,3,i+1)
  else
    subplot(2,3,i+2)
  end
  
  level = levels(i);
  [ mesh_xs, mesh_data ] = gridnd2mesh(gProj, dataProj);
  hV(i) = patch(isosurface(mesh_xs{:}, mesh_data, level));
  isonormals(mesh_xs{:}, mesh_data, hV(i));
  hV(i).FaceColor = color;
  hV(i).EdgeColor = 'none';
  hV(i).FaceAlpha = 1;
%   lighting phong
%   camlight left
%   camlight right
  view(3)
  hold on
    [ mesh_xs0, mesh_data0 ] = gridnd2mesh(gProj, data0Proj);
  hL(i) = patch(isosurface(mesh_xs0{:}, mesh_data0, level));
  isonormals(mesh_xs0{:}, mesh_data0, hL(i));
  hL(i).FaceColor = color;%[0.5 0.5 0.5];
  hL(i).EdgeColor = 'none';
  hL(i).FaceAlpha = .2;
  lighting phong
  camlight left
  camlight right
  axis([vf.g.min(1) vf.g.max(1) vf.g.min(2) vf.g.max(2) vf.g.min(3) vf.g.max(3)])
  axis square
  set(gca,'FontSize',textSize)
    xlabel('$x$','Interpreter','latex','FontSize',textSize)
  ylabel('$y$','Interpreter','latex','FontSize',textSize)
  zlabel('$z$','Interpreter','latex','FontSize',textSize)
  title(['Max Tracking Error = ' num2str(-level,3) ' m'],'FontSize',titleTextSize);
 
  view(3)

  end
end


end