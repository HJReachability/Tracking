classdef P4D_Q2D_Rel_3DTransform < DynSys
  properties
    % Tracker rotational velocity bounds
    wMin
    wMax
    
    % Tracker acceleration bounds
    aMax
    aMin
    
    % Planner bounds
    pMax
    
    %Disturbance bounds
    dMin
    dMax

    % active dimensions
    dims
  end
  
  methods
    function obj = P4D_Q2D_Rel_3DTransform(x, wMin, wMax, aMin, aMax,...
            pMax, dMin, dMax,  dims)
      % Dynamics:
      %     \dot x_1 = v - u{1}.*y - up1 - dx
      %     \dot x_2 = u{1}.*x - up2 - dy           
      %     \dot x_3 = u{2} - dz
      %         wMin <= u{1} <= wMax
      %         aMin <= u{2} <= aMax
      
      % u{1,2}  <- control of 4D plane (tracker)
      % d{2,4}  <- control of 2D quadrotor (planner)
      % d{1,3,5}  <- disturbance
      
      if nargin < 1 || isempty(x)
        x = zeros(obj.nx, 1);
      end
      
      if ~iscolumn(x)
        x = x';
      end      
      
      if nargin < 2
        wMin = -1;        
        wMax = 1;
      end
      
      if nargin < 4
          aMin = -1;        
        aMax = 1;
        
      end
      
      if nargin < 6
          pMax = .1;
      end
      
      if nargin < 7
        dMax = [0, 0];
        dMin = [0, 0];
      end
      
      if nargin < 9
        dims = 1:3;
      end
      
      obj.x = x;
      obj.xhist = x;
      
      obj.wMin = wMin;
      obj.wMax = wMax;

      obj.pMax = pMax;      
      
      obj.dMin = dMin;
      obj.dMax = dMax;
      
      obj.aMin = aMin;
      obj.aMax = aMax;      
      
      obj.dims = dims;
      obj.nx = length(dims);
      
      obj.nu = 2;
      obj.nd = 3;
      
      obj.pdim = [find(dims == 1) find(dims == 2)]; % Position dimensions
      obj.hdim = find(dims == 3); % angle dimensions
      obj.vdim = find(dims == 4); % velocity dimensions
    end
  end
end