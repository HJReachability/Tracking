% Script for saving figures of 2D projection of value function at a suitable
% level for automatically determining time-varying tracking error bound

[g2D, data2D] = proj(sD.grid, data, [0 0 1 1], 'max');
[g3D, data3D] = proj(sD.grid, data, [0 0 0 1], 'max');

numT = length(tau);

fig_filename = 'TEB_Q8D_Q4D_0.5';

level = -0.13;

TEB = nan(1,numT);

f = figure;
f.Color = 'white';
box on
grid on

h = cell(numT,1);

for i = 1:numT
  h{i} = visSetIm(g2D, data2D(:,:,i), 'r', level);
  hold on
  %   h = visSetIm(g2D, data2D(:,:,i), 'r', level, extraArg);
  
  if ~isempty(h{i}.ContourMatrix)
    indsToKeep = h{i}.ContourMatrix(1,:) ~= level;
    contourPts_x = h{i}.ContourMatrix(1,indsToKeep);
    TEB(i) = max(abs(contourPts_x));
  end
  
  
  % keyboard
end

savefig(sprintf('%s.fig', fig_filename));