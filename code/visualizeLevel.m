function [hCost, hValue, hV1, hL1] = visualizeLevel(g,data,type,cost,valExtraStates)
% inputs:
% g - grid
% data - data that has been computed with min w/ target
% type - quad or plane
% cost - square or circle (depending how you set the initial cost)
% valExtraStates - values at the non-position states

figure(3)
clf
colormap('winter');



if strcmp(type,'plane')
  
  %find terminal cost
data0 = shapeRectangleByCorners(g,[0 0 -Inf -Inf],[0 0 Inf Inf]);

%largest cost on the map
costMax = g.max(1);%max(data0(:));
costMin = g.min(1);%min(data0(:));

%project onto valExtraStates
[gProj, data0Proj] = proj(g, data0, [0 0 1 1], valExtraStates);

%plot cost
subplot(1,3,1)
hCost = surfc(gProj.xs{1},gProj.xs{2},data0Proj);
title(['Cost Function, v = ' num2str(valExtraStates(1)) ...
  ' m/s and \theta = ' num2str(valExtraStates(2)) ' rad']);
axis square

%get data
data = -data;
%data(data>costMax) = nan;

%project onto valExtraStates
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
  ' m/s and \theta = ' num2str(valExtraStates(2)) ' rad'])

hValue(2).ContourZLevel = levelMin - .05;
hValue(2).LevelList = hValue(2).LevelList(hValue(2).LevelList <= levelMax);

zlim([levelMin-.05 levelMax]);
caxis([levelMin levelMax]);
axis square


elseif strcmp(type,'quad')
  if strcmp(cost,'square')
    data0 = shapeRectangleByCorners(g,[0 -Inf 0 -Inf],[0 Inf 0 Inf]);
  elseif strcmp(cost,'circle')
    
    ignoreDims = [2,4];
    center = [0 0 0 0];
    data0 = zeros(g.shape);
    for i = 1 : g.dim
      if(all(i ~= ignoreDims))
        data0 = data0 + (g.xs{i} - center(i)).^2;
      end
    end
    data0 = sqrt(data0);
  end
  
  %largest cost on the map
  costMax = g.max(1);%max(data0(:));
  costMin = g.min(1);%min(data0(:));
  
  %project cost onto valExtraStates
  [gProj, data0Proj] = proj(g, data0, [0 1 0 1], valExtraStates);
 
  
  %project data onto valExtraStates
  data = -data; %make positive so contour maps can understand
  [~, dataProj] = proj(g,data,[0 1 0 1],valExtraStates);
  
  %Find a few good levels to plot for next section
  levelMin = max(min(dataProj(:)), costMin);
  levelMax = min(max(dataProj(:)),costMax);
  levels = linspace(levelMin,levelMax,8);
  levels = levels([2,4,6]);
  
  %plot
  subplot(1,3,2)
  hValue = surfc(gProj.xs{1},gProj.xs{2},dataProj);
  title(['Value Function, v_x = ' num2str(valExtraStates(1)) ...
    ' m/s and v_y = ' num2str(valExtraStates(2)) ' m/s'])
  
  hValue(2).ContourZLevel = levelMin - .05;
  hValue(2).LevelList = hValue(2).LevelList(hValue(2).LevelList <= levelMax);
  
  zlim([levelMin-.05 levelMax]);
  caxis([levelMin levelMax]);
  axis square
  
    %plot cost
   subplot(1,3,1)
  hCost = surfc(gProj.xs{1},gProj.xs{2},data0Proj);
  title(['Cost Function, v_x = ' num2str(valExtraStates(1)) ...
    ' m/s and v_y = ' num2str(valExtraStates(2)) ' m/s']);
  
  hCost(2).LevelList = hCost(2).LevelList(hCost(2).LevelList <= levelMax);
  
  zlim([0 levelMax]);
  caxis([0 levelMax]);
  axis square
end




subplot(1,3,3)
[~,hV1]=contour(gProj.xs{1},gProj.xs{2},dataProj,levels,...
  'Linestyle','--','LineWidth',2);
hold on
[~,hL1] = contour(gProj.xs{1},gProj.xs{2},data0Proj,levels,...
  'LineWidth',2);
axis([g.min(1) g.max(1) g.min(3) g.max(3)])
title('Mapping initial state to tracking error bound')
axis square
set(gcf,'Color','white')
colorbar
end