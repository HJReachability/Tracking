function TEB_ind = get_TEB_ind(tau, sD, data, rel_x, TEB, min_level)
% Determines the last index from which to create TEB_list from TEB by
% finding the smallest index of data such that rel_x evaluates to a 
% negative value

    max_ind = length(tau);
    min_ind = 1;
    TEB_ind = ceil(max_ind/2);
    while max_ind > min_ind
    if eval_u(sD.grid, data(:,:,:,:,TEB_ind), rel_x) >= min_level
      % If inside the TEB for the current index, look for smaller TEB
      min_ind = TEB_ind;                 % Set minimum ind to current ind
      TEB_ind = ceil((max_ind + TEB_ind)/2); % Set current ind to between min and max ind

    else
      % If outside the TEB for the current index, looking for larger TEB
      max_ind = TEB_ind - 1; % Set maximum ind to current ind (exclude current ind)
      TEB_ind = floor((min_ind + TEB_ind)/2);
    end
    end

    TEB_list = flip(TEB(1:TEB_ind-1));
end
