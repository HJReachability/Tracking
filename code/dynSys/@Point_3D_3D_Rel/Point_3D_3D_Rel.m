classdef Point_3D_3D_Rel < DynSys
  properties
    % Control bounds
    uMin
    uMax
    
    % Planner bounds
    pMin
    pMax
    
    % active dimensions
    dims
  end
  
  methods
    function obj = Point_3D_3D_Rel(x, uMin, uMax, pMin, pMax, dims)
      % obj = P3D_Q2D_Rel(x, uMin, uMax, pMin, pMax, dMin, dMax, dims)
      %     Constructor for a 3D plane relative to a 2D quadrotor
      %
      % Dynamics:
      %     \dot x_1 = u{1}  - d{2}
      %     \dot x_2 = u{2}  - d{4}               
      %     \dot x_3 = u{3} - d{3}
      %         uMin <= u <= uMax
      
      % u       <- control of 3D (tracker)
      % d       <- control of 3D (planner)
      
      if nargin < 1 || isempty(x)
        x = zeros(obj.nx, 1);
      end
      
      if ~iscolumn(x)
        x = x';
      end      
      
      if nargin < 2
        uMin = -1*ones(3,1);        
        uMax = 1*ones(3,1);
      end
      
      if nargin < 4
        pMin = -.5*ones(3,1);
        pMax = .5*ones(3,1);
      end
      
      if nargin < 6
        dims = 1:3;
      end
      
      obj.x = x;
      obj.xhist = x;
      
      obj.uMin = uMin;
      obj.uMax = uMax;

      obj.pMin = pMin;
      obj.pMax = pMax;      

      obj.dims = dims;
      obj.nx = length(dims);
      
      obj.nu = 3;
      obj.nd = 3;
      
    end
  end
end