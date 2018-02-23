% Script for saving figures of 2D projection of value function at a suitable
% level for automatically determining time-varying tracking error bound

function TEB = tvTEB(numD, fig_filename, level, sD, data, tau)

if numD ~= 2 && numD ~= 3
  return
end

numT = length(tau);

if numD == 2
  [g2D, data2D] = proj(sD.grid, data, [0 0 1 1], 'max');
  colors = jet(numT);
  h = cell(numT,1);
else
  [g3D, data3D] = proj(sD.grid, data, [0 0 1 0], 'max');
end


TEB = nan(1,numT);

f = figure;
f.Color = 'white';
f.Position = [100 100 800 600];
box on
grid on


for i = 1:numT
  if numD == 2
    
    if i == 1
      
    end
    h{i} = visSetIm(g2D, data2D(:,:,i), colors(i,:), level);
    
    hold on
    %   h = visSetIm(g2D, data2D(:,:,i), 'r', level, extraArg);
    
    if ~isempty(h{i}.ContourMatrix)
      indsToKeep = h{i}.ContourMatrix(1,:) ~= level;
      contourPts_x = h{i}.ContourMatrix(1,indsToKeep);
      TEB(i) = max(abs(contourPts_x));
    end
    
  else
    visSetIm(g3D, data3D(:,:,:,i), 'r', level);
    export_fig(sprintf('%s_%d', fig_filename, i), '-png', '-m2')
    
    clf
  end
  
  
end

if numD == 2
  savefig(sprintf('%s.fig', fig_filename));
end
end