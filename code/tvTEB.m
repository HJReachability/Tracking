% Script for saving figures of 2D projection of value function at a suitable
% level for automatically determining time-varying tracking error bound

[g2D, data2D] = proj(sD.grid, data, [0 0 1 1], 'max');

dataSize = size(data2D);
numT = dataSize(end);

fig_filename = 'TEB_aMax1.00';

level = -1.5950;

TEB = nan(1,numT);

for i = 1:numT
  extraArg.fig_filename = sprintf('%s_%d', fig_filename, i);
  f = figure;
  f.Color = 'white';
  box on
  grid on
  
  h = visSetIm(g2D, data2D(:,:,i), 'r', level, extraArg);
  
  if ~isempty(h.ContourMatrix)
    indsToKeep = h.ContourMatrix(1,:) ~= level;
    contourPts_x = h.ContourMatrix(1,indsToKeep);
    TEB(i) = max(abs(contourPts_x));
  end
  
  savefig(sprintf('%s.fig', extraArg.fig_filename));
end