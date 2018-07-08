function dx = dynamics(obj, ~, x, u, d)
      % Dynamics:
      %     \dot x_1 = v + u{1}.*y - d1 - d2
      %     \dot x_2 = -u{1}.*x - d3 - d4           
      %     \dot x_3 = u{2} - d5
      %         wMin <= u{1} <= wMax
      %         aMin <= u{2} <= aMax
      
      % u{1,2}  <- control of 4D plane (tracker)
      % d{2,4}  <- control of 2D quadrotor (planner)
      % d{1,3,5}  <- disturbance

dx = cell(obj.nx,1);
dims = obj.dims;

returnVector = false;
if ~iscell(x)
  returnVector = true;
  x = num2cell(x);
  u = num2cell(u);
  d = num2cell(d);
end

for i = 1:length(dims)
  dx{i} = dynamics_cell_helper(obj, x, u, d, dims, dims(i));
end

if returnVector
  dx = cell2mat(dx);
end

end

function dx = dynamics_cell_helper(obj, x, u, d, dims, dim)
switch dim
  case 1
    %dx = v - w.*y - d1 - d2
    dx = x{3}+ u{1}.*x{2} - d{1} + d{2};
  case 2
    %dx = w.*x - d3 - d4
    dx = -u{1}.*x{1} - d{3} + d{4};
  case 3
    dx = u{2} - d{5};
  otherwise
    error('Only dimension 1-3 are defined for dynamics of %s!', class(obj))
end
end
