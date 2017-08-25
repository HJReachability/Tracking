classdef Q6D < DynSys
  properties
    uMin        % Control bounds (3x1 vector)
    uMax
    
    dMin
    dMax

    % Constants
    grav = 9.81    % Acceleration due to gravity (for convenience)
    
    % active dimensions
    dims
  end
  
  methods
    function obj = Q6D(x, uMin, uMax, dMin, dMax, dims)
        % obj = Q6D(x, uMin, uMax, dMin, dMax, pMin, pMax, dims)
        %     Constructor for a 6D quadrotor
        %
        %     Dynamics of the 6D Quadrotor
        %     \dot x_1 = x_2 - d(1)
        %     \dot x_2 = g * tan(u(1))
        %     \dot x_3 = x_4 - d(2)
        %     \dot x_4 = g * tan(u(2))
        %     \dot x_5 = x_6 - d(3)
        %     \dot x_6 = u(3) - g
        %     min (radians)      <=     [u(1); u(2)]   <= max (radians)
        %     min thrust (m/s^2) <=         u(3)       <= max thrust (m/s^2)
        %     dist vmin (m/s)    <= [d(1); d(3); d(5)] <= dist vmax (m/s)
      
        if nargin < 1
            x = zeros(obj.nx, 1);
        end
        
        if nargin < 3
            angleMax = deg2rad(15);
            uMin = [-angleMax; -angleMax; 4];
            uMax = [angleMax; angleMax; 16];
        end
        
        if nargin < 5
            dMin = zeros(1,3);
            dMax = zeros(1,3);
        end
        
        
        if nargin < 6
            dims = 1:2;
        end
        
        obj.x = x;
        obj.xhist = x;
        
        obj.uMax = uMax;
        obj.uMin = uMin;
        obj.dMax = dMax;
        obj.dMin = dMin;
        
        obj.dims = dims;
        obj.nx = length(dims);
        obj.nu = 3;
        obj.nd = 3;
        obj.pdim = [1 3 5];
        obj.vdim = [2 4 6];
    end
  end
end