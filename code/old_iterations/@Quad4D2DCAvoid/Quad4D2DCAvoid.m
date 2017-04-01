classdef Quad4D2DCAvoid < DynSys
  
  properties
    uMax
    uMin
    dMax
    dMin
    dims % dimensions that are active
    
  end
  
  methods
    function obj = Quad4D2DCAvoid(x, uMax, uMin, dMax, dMin, dims)
      % obj = Quad4DCAvoid(x, aMax, bMax)
      %     aMax: x- and y-acceleration bound for vehicle A
      %     bMax: x- and y-acceleration bounds for vehicle B
      %
      % Dynamics:
      %     \dot{x}_1 = x_2 - uB(1) + dx
      %     \dot{x}_2 = uA(1)
      %     \dot{x}_3 = x_4 -uB(2) + dy
      %     \dot{x}_4 = uA(2)
      %       aMin(i) <= uA(i) <= aMax(i)
      %       bMin(i) <= uB(i) <= bMax(i), i = 1,2
      
      if ~iscolumn(x)
        x = x';
      end      
      
      if nargin < 1
        x = zeros(obj.nx, 1);
      end
      
      
      if nargin < 5
        dMax = 0;
        dMin = 0;
      end
      
      if nargin < 6
        dims = [1 2 3 4];
      end
      
      
      obj.x = x;
      obj.xhist = obj.x;
      
      obj.uMax = uMax;
      obj.uMin = uMin;
      obj.dMax = dMax;
      obj.dMin = dMin;
      
      obj.dims = dims;
      obj.nx = length(dims);
      
      obj.pdim = [find(dims == 1) find(dims == 3)]; % Position dimensions
      obj.vdim = [find(dims == 2) find(dims == 4)]; % Velocity dimensions

      obj.nu = 4;
      obj.nd = 2;

      
      if numel(x) ~= obj.nx
        error('Initial state does not have right dimension!');
      end
      

      

    end
  end
end