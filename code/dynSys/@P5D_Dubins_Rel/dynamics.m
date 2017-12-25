function dx = dynamics(obj, t, x, u, d, ~)
% function dx = dynamics(t, x, u)
%     Dynamics of the relative system between Plane5D and DubinsCar

%% For reachable set computations
if iscell(x)
  dx = cell(3,1);
  
  % States
  xr = x{1};
  yr = x{2};
  tr = x{3};
  v  = x{4};
  w  = x{5};
  
  % Controls
  a     = u{1}; % Linear acceleration
  alpha = u{2}; % Angular acceleration
  
  % Disturbances
  wOther = d{5}; % Turn rate of other vehicle (at origin)
  
  dx{1} = -obj.vOther + v * cos(tr) + wOther * yr + d{1};
  dx{2} = v * sin(tr) - wOther * xr + d{2};
  dx{3} = w - wOther;
  dx{4} = a + d{3};
  dx{5} = alpha + d{4};
end

% %% For simulations
% if isnumeric(x)
%   dx = zeros(3,1);
%   
%   dx(1) = -obj.va + obj.vb * cos(x(3)) + u*x(2);
%   dx(2) = obj.vb * sin(x(3)) - u*x(1);
%   dx(3) = d - u;
% end

end
