function [hCostS, hCostC, hValueS, hValueC, hV, hL] = visualize10D(g,data,data0, viewSlice, cost,valExtraStates,fig)
figure(fig)
clf
colormap('winter');



%viewSlice = 'velocity';
%% Quad10D

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

if strcmp(viewSlice, 'position')
  range_lower([1 5 9]) = [vfs.gs{1}.min(1); vfs.gs{2}.min(1); vfs.gs{3}.min(1)]-small;
  range_upper([1 5 9]) = [vfs.gs{1}.max(1); vfs.gs{2}.max(1); vfs.gs{3}.max(1)]+small;
elseif strcmp(viewSlice,'velocity')
  range_lower([2 6 10]) = [vfs.gs{1}.min(2); vfs.gs{2}.min(2); vfs.gs{3}.min(2)]-small;
  range_upper([2 6 10]) = [vfs.gs{1}.max(2); vfs.gs{2}.max(2); vfs.gs{3}.max(2)]+small;
elseif strcmp(viewSlice,'x')
  range_lower([1 2]) = [vfs.gs{1}.min(1); vfs.gs{1}.min(2)]-small;
  range_upper([1 2]) = [vfs.gs{1}.max(1); vfs.gs{1}.max(2)]+small;
elseif strcmp(viewSlice,'XYVy')
  range_lower([1 5 6]) = [vfs.gs{1}.min(1); vfs.gs{2}.min(1); vfs.gs{2}.min(2)]-small;
  range_upper([1 5 6]) = [vfs.gs{1}.max(1); vfs.gs{2}.max(1); vfs.gs{2}.max(2)]+small;
else
  error('what viewSlice?')
end

vf = reconSC(vfs, range_lower, range_upper, 0, 'min');

vfs0 = vfs;
vfs0.datas = data0;
vf0 = reconSC(vfs0, range_lower, range_upper, 0, 'min');
gProj = vf.g;

if strcmp(cost,'quadratic_decomp')
  dataProj = -sqrt(-vf.data);
  data0Proj = -sqrt(-vf0.data);
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
  levels = levels([end-40, end-20, end-1]);
  
  %plot
  subplot(2,3,4)
  if strcmp(viewSlice,'x')
    gProj2D = gProj;
    dataProj2D = dataProj;
  else
    [gProj2D, dataProj2D] = proj(gProj, dataProj, [0 0 1], 0);
  end
  hValueS = surf(gProj2D.xs{1},gProj2D.xs{2},dataProj2D);
  hold on
  [~, hValueC] = contour(gProj2D.xs{1},gProj2D.xs{2},dataProj2D,levels,...
    'LineWidth',2);
  set(gca,'FontSize',textSize)
  title('Value Function','FontSize',titleTextSize)
  %title(['Value Function, v_x = ' num2str(valExtraStates(1)) ...
  %  ' m/s, v_y = ' num2str(valExtraStates(2)) ' m/s' ...
  %  'v_z = ' num2str(valExtraStates(2)) ' m/s'],'FontSize',15)
  
  %hValueC.ContourZLevel = levelMax + .05;
  hValueC.LevelList = hValueC.LevelList(hValueC.LevelList >= levelMin);
  
  zlim([levelMin 0]);%levelMax + .05]);
  caxis([levelMin levelMax]);
  axis square
  zlabel('$V(s)$','Interpreter','latex','FontSize',textSize)
    if strcmp(viewSlice,'position')
  xlabel('$x$','Interpreter','latex','FontSize',textSize)
  ylabel('$y$','Interpreter','latex','FontSize',textSize)
  elseif strcmp(viewSlice,'velocity')
    xlabel('$v_x$','Interpreter','latex','FontSize',textSize)
    ylabel('$v_y$','Interpreter','latex','FontSize',textSize)
  elseif strcmp(viewSlice,'XVx')
    xlabel('$x$','Interpreter','latex','FontSize',textSize)
    ylabel('$v_x$','Interpreter','latex','FontSize',textSize)
  elseif strcmp(viewSlice,'XYVy')
    xlabel('$x$','Interpreter','latex','FontSize',textSize)
    ylabel('$y$','Interpreter','latex','FontSize',textSize)
  end
  
  %   %plot cost
  if strcmp(viewSlice,'x')
    data0Proj2D = data0Proj;
  else
    [~, data0Proj2D] = proj(gProj, data0Proj, [0 0 1], 0);
  end
  
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
  
  zlim([levelMin 0]);
  caxis([levelMin 0]);
  axis square
  
    zlabel('$l(s)$','Interpreter','latex','FontSize',textSize)
  if strcmp(viewSlice,'position')
  xlabel('$x$','Interpreter','latex','FontSize',textSize)
  ylabel('$y$','Interpreter','latex','FontSize',textSize)
  elseif strcmp(viewSlice,'velocity')
    xlabel('$v_x$','Interpreter','latex','FontSize',textSize)
    ylabel('$v_y$','Interpreter','latex','FontSize',textSize)
  elseif strcmp(viewSlice,'XVx')
    xlabel('$x$','Interpreter','latex','FontSize',textSize)
    ylabel('$v_x$','Interpreter','latex','FontSize',textSize)
  elseif strcmp(viewSlice,'XYVy')
    xlabel('$x$','Interpreter','latex','FontSize',textSize)
    ylabel('$y$','Interpreter','latex','FontSize',textSize)
  end
  
  colorbar
  
  if strcmp(viewSlice,'XVx')
subplot(2,3,2)
  [~,hV]=contour(gProj.xs{1},gProj.xs{2},dataProj,levels,...
    'Linestyle','--','LineWidth',2);
  hold on
  [~,hL] = contour(gProj.xs{1},gProj.xs{2},data0Proj,levels,...
    'LineWidth',2);
  axis([gProj.min(1) gProj.max(1) gProj.min(2) gProj.max(2)])
  title('Mapping initial state to tracking error bound','FontSize',15)
  axis square
  set(gcf,'Color','white')
  colorbar
  xlabel('$x$','Interpreter','latex','FontSize',20)
  ylabel('$v_x$','Interpreter','latex','FontSize',20)
  else
    
  for i = 1:length(levels)
  color = cmap(16*i,:);
  
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
  
    if strcmp(viewSlice,'position')
  xlabel('$x$','Interpreter','latex','FontSize',textSize)
  ylabel('$y$','Interpreter','latex','FontSize',textSize)
  zlabel('$z$','Interpreter','latex','FontSize',textSize)
  elseif strcmp(viewSlice,'velocity')
    xlabel('$v_x$','Interpreter','latex','FontSize',textSize)
    ylabel('$v_y$','Interpreter','latex','FontSize',textSize)
    zlabel('$v_z$','Interpreter','latex','FontSize',textSize)
  elseif strcmp(viewSlice,'XYVy')
    xlabel('$x$','Interpreter','latex','FontSize',textSize)
    ylabel('$y$','Interpreter','latex','FontSize',textSize)
    zlabel('$v_y$','Interpreter','latex','FontSize',textSize)
  end
  
  title(['Max Tracking Error = ' num2str(-level,3) ' m'],'FontSize',titleTextSize);
 
  view(3)

  end
  end
end