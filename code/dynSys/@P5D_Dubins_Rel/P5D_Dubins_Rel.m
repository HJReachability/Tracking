classdef P5D_Dubins_Rel < DynSys
  properties
    % Control bounds of this vehicle
    aRange   % Linear acceleration
    alphaMax % Angular acceleration 
    
    % Control bounds of other vehicle (at origin)
    wMax     % Turn rate
    
    % Disturbance bounds
    dMax     % 4D
    
    % Vehicle speeds
    vOther
    
    dims

  end % end properties
 
  methods
    function obj = P5D_Dubins_Rel(x, aRange, alphaMax, vOther, wMax, dMax, dims)
      %% Process initial state
      obj.x = x;
      obj.xhist = x;
  
      %% Process control range
      if nargin < 2
        aRange = [-0.15; 0.15];
      end
      
      if nargin < 3
        alphaMax = 3;
      end
          
      if nargin < 4
        vOther = 0.1;
      end
      
      if nargin < 5
        wMax = 2;
      end
      
      if nargin < 6
        dMax = [0.02; 0.02; 0.2; 0.02];
      end
      
      if nargin < 7
        dims = 1:5;
      end
      
      obj.aRange = aRange;
      obj.alphaMax = alphaMax;
      obj.vOther = vOther;
      obj.wMax = wMax;
      obj.dMax = dMax;
      
      obj.dims = dims;
      
      obj.nx = 5;
      obj.nu = 2;
      obj.nd = 5;
      
    end % end constructor
  end % end methods
end % end class