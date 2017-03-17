function coords = get_obs_coords_for_plot(obstacles)
%       obstacles(isinf(obstacles)) = [];
coords = cell(3,1);
for i = 1:3
  coords{i} = squeeze(obstacles(:,i,:));
end
end