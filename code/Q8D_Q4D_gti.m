function [indX, indY, TEB_list] = Q8D_Q4D_gti(s, p, g, vf, level, TEB)
% [indX, indY, TEB_list] = Q8D_Q4D_gti(s, p, tau, g, vf, level)
%     Get tracking error bound index (gti) and TEB list given the current
%     states of the tracker and planner
%
% Inputs:
%     s     - tracker state
%     p     - planner state
%     g     - grid on which vf is represented
%     vf    - value function
%     level - level of value function to compare to
%
% Outputs:
%     indX     - tracking error bound X index
%     indY     - tracking error bound Y index
%     TEB_list - list of tracking error bounds corresponding
%
% Mo Chen, 2018-02-19

% Constants
XDims = 1:4;
YDims = 5:8;

% Relative state
r = Q8D_Q4D_grs(s, p);

indX = Q8D_Q4D_gti_single(g, vf, r(XDims), level);
indY = Q8D_Q4D_gti_single(g, vf, r(YDims), level);

TEB_ind = min(indX, indY);
TEB_list = flip(TEB(1:TEB_ind-1));
end

function [TEB_ind, values] = Q8D_Q4D_gti_single(g, vf, r, level)
% [TEB_ind, values] = Q8D_Q4D_gti_single(tau, g, vf, r, level)
%     Determines the last index from which to create TEB_list from TEB by
%     finding the smallest index of vf such that r evaluates to a value greater
%     than or equal to level, i.e. arg min_i vf(i,r) >= level, where i is a
%     time index

tau_length = size(vf, 5);

if nargout > 1
  values = zeros(1, tau_length);
  for i = 1:tau_length
    values(i) = eval_u(g, vf(:,:,:,:,i), r);
  end
end

max_ind = tau_length;
min_ind = 1;
TEB_ind = ceil(max_ind/2);
while max_ind > min_ind
  if eval_u(g, vf(:,:,:,:,TEB_ind), r) >= level
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
