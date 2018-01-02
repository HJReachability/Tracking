function dx = dynamics(obj, ~, x, u, d)
      % Dynamics:
      %     \dot x_1 = (x_4)*cos(x_3)  - d{1}  - d{2}
      %     \dot x_2 = (x_4)*sin(x_3)  - d{3}  - d{4}               
      %     \dot x_3 = u{1}
      %     \dot x_4 = u{2}
      %         uMin <= u{1} <= uMax
      %         aMin <= u{2} <= aMax
      
      % u{1,2}  <- control of 4D plane (tracker)
      % d{2,4}  <- control of 2D quadrotor (planner)
      % d{1,3}  <- disturbance

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
    %dx = (x{dims==4}).*cos(x{dims==3}) - d{1} - d{2};
    dx = (x{4}).*cos(x{3}) - d{1} - d{2};
  case 2
    %dx = (x{dims==4}).*sin(x{dims==3}) - d{3} - d{4};
    dx = (x{4}).*sin(x{3}) - d{3} - d{4};
  case 3
    dx = u{1};
  case 4
    dx = u{2};
  otherwise
    error('Only dimension 1-4 are defined for dynamics of %s!', class(obj))
end
end
