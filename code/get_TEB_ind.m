function [TEB_ind, values] = get_TEB_ind(tau, sD, data, rel_x, min_level)
% TEB_list = get_TEB_list(tau, sD, data, rel_x, TEB, min_level)
%
% Determines the last index from which to create TEB_list from TEB by
% finding the smallest index of data such that rel_x evaluates to a
% a value greater than or equal to min_level

if nargin > 1
  values = zeros(1,length(tau));
  for i = 1:length(tau)
    values(i) = eval_u(sD.grid, data(:,:,:,:,i), rel_x);
  end
end

max_ind = length(tau);
min_ind = 1;
TEB_ind = ceil(max_ind/2);
while max_ind > min_ind
  if eval_u(sD.grid, data(:,:,:,:,TEB_ind), rel_x) >= min_level;
    % If inside the TEB for the current index, look for smaller TEB
    min_ind = TEB_ind;                 % Set minimum ind to current ind
    TEB_ind = ceil((max_ind + TEB_ind)/2); % Set current ind to between min and max ind
    
  else
    % If outside the TEB for the current index, looking for larger TEB
    max_ind = TEB_ind - 1; % Set maximum ind to current ind (exclude current ind)
    TEB_ind = floor((min_ind + TEB_ind)/2);
  end
end


end
