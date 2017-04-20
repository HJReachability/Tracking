function out = break_down_obs(in, max_size_out)
% out = break_down_obs(in)
%     Breaks down a large piece of obstacle into smaller pieces
%
% Inputs:
%     in           - input obstacle (single obstacle)
%     max_size_out - maximum size of output obstacles
%
% Output:
%     out          - output obstacles

% initialization
out = [];

num_obs = size(in, 3);

for k = 1:num_obs
  obs_k = in(:,:,k);
  ranges_in = zeros(2,2);
  const_dim = false(3,1);
  iter = 0;
  
  % Get constant dimension and value, and obstacle ranges
  for j = 1:3
    if max(obs_k(:, j)) - min(obs_k(:, j)) > 0
      iter = iter + 1;
      ranges_in(iter,:) = [min(obs_k(:, j)) max(obs_k(:, j))];
    else
      const_dim(j) = true;
      const_val = min(obs_k(:, j));
    end
  end
  
  % Iterate through two non-constant dimensions and add obstacles
  lower = ranges_in(:,1);
  upper = lower + max_size_out;
  
  while lower(1) < ranges_in(1,2)
    while lower(2) < ranges_in(2,2)
      next_obs = zeros(4,3);
      next_obs(:, const_dim) = const_val;
      
      next_obs(1, ~const_dim) = [lower(1) lower(2)];
      next_obs(2, ~const_dim) = [lower(1) min(upper(2), ranges_in(2,2))];
      next_obs(3, ~const_dim) = [min(upper(1), ranges_in(1,2)), ...
        min(upper(2), ranges_in(2,2))];
      next_obs(4, ~const_dim) = [min(upper(1), ranges_in(1,2)), lower(2)];
      
      lower(2) = lower(2) + max_size_out;
      upper(2) = upper(2) + max_size_out;
      
      out = cat(3, out, next_obs);
    end
    
    lower(1) = lower(1) + max_size_out;
    lower(2) = ranges_in(2,1);
    upper = lower + max_size_out;
  end
end
end