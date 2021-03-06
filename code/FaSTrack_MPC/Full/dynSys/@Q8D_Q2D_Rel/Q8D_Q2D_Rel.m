classdef Q8D_Q2D_Rel < DynSys
  properties
    uMin        % Q8D Control bounds (3x1 vector)
    uMax
    
    pMin        % Q2D Planner control bounds
    pMax
    
    dMin        % disturbance (wind) velocity bounds
    dMax        
    
    % Constants
    %   The choices of n0, d1, d0 actually results in a very large
    %   steady state error in the pitch/roll; this seems to be
    %   expected according to Pat's report
    n0 = 10     % Angular dynamics parameters
    d1 = 8
    d0 = 10
    
    g = 9.81    % Acceleration due to gravity (for convenience)
    m = 1.3     % Mass
    
    % active dimensions
    dims
  end
  
  methods
    function obj = Q8D_Q2D_Rel(x, uMin, uMax, pMin, pMax, dMin, dMax, dims)
      % obj = Q8D_Q2D_Rel(x, uMin, uMax, pMin, pMax, dMin, dMax, dims)
      %     Constructor for a 8D quadrotor relative to a 2D quadrotor
      %
      % Dynamics:
      %     \dot x_1 = x_2                      + d{1}  - d{2}
      %     \dot x_2 = g*tan(x_3)               
      %     \dot x_3 = -d1*x_3 + x_4
      %     \dot x_4 = -d0*x_3       + n0*u{1}
      %     \dot x_5 = x_6                      + d{3}  - d{4}
      %     \dot x_6 = g*tan(x_7)               
      %     \dot x_7 = -d1*x_7 + x_8
      %     \dot x_8 = -d0*x_7       + n0*u{2}
      %         uMin <= [u1; u2; u3] <= uMax
      
      % u       <- control of 8D quadrotor (tracker)
      % d{2,4}  <- control of 2D quadrotor (planner)
      % d{1,3}  <- disturbance
      
      if nargin < 1 || isempty(x)
        x = zeros(obj.nx, 1);
      end
      
      if ~iscolumn(x)
        x = x';
      end      
      
      if nargin < 2
        uMin = [-10/180*pi; -10/180*pi];        
        uMax = [10/180*pi; 10/180*pi];
      end
      
      if nargin < 4
        pMin = [-1; -1];
        pMax = [1; 1];
      end
      
      if nargin < 6
        dMax = [0.1; 0.1];
        dMin = [-0.1; -0.1];
      end
      
      if nargin < 7
        dims = 1:4;
      end
      
      obj.x = x;
      obj.xhist = x;
      
      obj.uMin = uMin;
      obj.uMax = uMax;

      obj.pMin = pMin;
      obj.pMax = pMax;      
      
      obj.dMin = dMin;
      obj.dMax = dMax;
      
      
      obj.dims = dims;
      obj.nx = length(dims);
      
      obj.nu = 2;
      obj.nd = 4;
      
      obj.pdim = [find(dims == 1) find(dims == 5)]; % Position dimensions
      obj.vdim = [find(dims == 2) find(dims == 6)]; % Velocity dimensions
    end
  end
end