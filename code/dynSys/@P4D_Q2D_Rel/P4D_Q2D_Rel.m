classdef P4D_Q2D_Rel < DynSys
  properties
    % Control bounds
    uMin
    uMax
    
    % Planner/"Human" bounds
    pxMax
    pyMax
    
    %Disturbance bounds
    dMin
    dMax
    
    % Tracker acceleration bounds
    aMax
    aMin
    
    % active dimensions
    dims
  end
  
  methods
    function obj = P4D_Q2D_Rel(x, uMin, uMax, pxMax, pyMax, dMin, dMax, aMin, aMax, dims)
      % obj = P4D_Q2D_Rel(x, uMin, uMax, pxMax, pyMax, dMin, dMax, dims)
      %     Constructor for a 4D plane relative to a 2D quadrotor
      %
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
      
      if nargin < 1 || isempty(x)
        x = zeros(obj.nx, 1);
      end
      
      if ~iscolumn(x)
        x = x';
      end      
      
      if nargin < 2
        uMin = -1;        
        uMax = 1;
      end
      
      if nargin < 4
        pxMax = 1;
        pyMax = 1;
      end
      
      if nargin < 6
        dMax = 0.1;
        dMin = -0.1;
      end
      
      if nargin < 8
        aMin = -1;        
        aMax = 1;
      end
      
      if nargin < 10
        dims = 1:4;
      end
      
      obj.x = x;
      obj.xhist = x;
      
      obj.uMin = uMin;
      obj.uMax = uMax;

      obj.pxMax = pxMax;
      obj.pyMax = pyMax;      
      
      obj.dMin = dMin;
      obj.dMax = dMax;
      
      obj.aMin = aMin;
      obj.aMax = aMax;      
      
      obj.dims = dims;
      obj.nx = length(dims);
      
      obj.nu = 2;
      obj.nd = 4;
      
      obj.pdim = [find(dims == 1) find(dims == 2)]; % Position dimensions
      obj.hdim = find(dims == 3); % angle dimensions
      obj.vdim = find(dims == 4); % velocity dimensions
    end
  end
end