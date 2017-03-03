function [hL, hV] = visualizeQuad10D(g,data,data0,cost,valExtraStates,fig)

%% Quad10D
figure(fig)
clf
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

vfs0 = vfs;
vfs0.datas = data0;
vf0 = reconSC(vfs0, range_lower, range_upper, 0, 'min');
gProj = vf.g;

if strcmp(cost,'quadratic_decomp')
  dataProj = -sqrt(-vf.data);
  data0Proj = -sqrt(-vf0.data);
elseif strcmp(cost,'oneNorm')
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
  levels = linspace(levelMin,levelMax,15);
  levels = levels([end-7, end-4, end-1]);
  
for i = 1:length(levels)
  color = cmap(16*i,:);
  subplot(1,length(levels),i)
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
    xlabel('$x$','Interpreter','latex','FontSize',20)
  ylabel('$y$','Interpreter','latex','FontSize',20)
  zlabel('$z$','Interpreter','latex','FontSize',20)
 
  view(3)

end


end