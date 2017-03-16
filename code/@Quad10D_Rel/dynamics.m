function dx = dynamics(obj, ~, x, u, d)
% dx = dynamics(obj, ~, x, u, ~)
%     Dynamics of the 10D Quadrotor
%         \dot x_1 = x_2
%         \dot x_2 = g * tan(x_3)
%         \dot x_3 = -d1 * x_3 + x_4
%         \dot x_4 = -d0 * x_3 + n0 * u1
%         \dot x_5 = x_6
%         \dot x_6 = g * tan(x_7)
%         \dot x_7 = -d1 * x_7 + x_8
%         \dot x_8 = -d0 * x_7 + n0 * u2
%         \dot x_9 = x_10 + d(1)
%         \dot x_10 = kT * u3 - obj.g
%              uMin <= [u1; u2; u3] <= uMax

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
    dx = x{dims==2} - u{1} + d{1} ;
  case 2
    dx = obj.g * tan(x{dims==3});
  case 3
    dx = -obj.d1 * x{dims==3} + x{dims==4};
  case 4
    dx = -obj.d0 * x{dims==3} + obj.n0 * u{2};
  case 5
    dx = x{dims==6} - u{3} + d{2} ;
  case 6
    dx = obj.g * tan(x{dims==7});
  case 7
    dx = -obj.d1 * x{dims==7} + x{dims==8};
  case 8
    dx = -obj.d0 * x{dims==7} + obj.n0 * u{4};
  case 9
    dx = x{dims==10} - u{5} + d{3};
  case 10
    dx = obj.kT * u{6} - obj.g;
    
  otherwise
    error('Only dimension 1-10 are defined for dynamics of Quad10D!')
end
end
