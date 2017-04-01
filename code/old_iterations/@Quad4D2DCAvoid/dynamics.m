function dx = dynamics(obj, ~, x, u, d)
% Dynamics of the Quad4DCAvoid, two quadrotors performing collision avoidance
      % Dynamics:
      %     \dot{x}_1 = x_2 - uB(1) + dx
      %     \dot{x}_2 = uA(1)
      %     \dot{x}_3 = x_4 -uB(2) + dy
      %     \dot{x}_4 = uA(2)
      %       aMin(i) <= uA(i) <= aMax(i)
      %       bMin(i) <= uB(i) <= bMax(i), i = 1,2

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
  dx{i} = dynamics_cell_helper(x, u, d, dims, dims(i));
end

if returnVector
  dx = cell2mat(dx);
end
end


function dx = dynamics_cell_helper(x, u, d, dims, dim)

switch dim
  case 1
    dx = x{dims==2} - u{1} + d{1};
  case 2
    dx = u{2};
  case 3
    dx = x{dims==4} - u{3} + d{2};
  case 4
    dx = u{4};
  otherwise
    error('Only dimensions 1-4 are defined for dynamics of Quad4D2DCAvoid!')
end
end