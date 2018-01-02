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
        %     \dot x_1 = x_4 - d(1)
        %     \dot x_2 = x_5 - d(2)
        %     \dot x_3 = x_6 - d(3)
        %     \dot x_4 = g * tan(u(1))
        %     \dot x_5 = g * tan(u(2))
        %     \dot x_6 = u(3) - g
        %     min (radians)      <=     [u(1); u(2)]   <= max (radians)
        %     min thrust (m/s^2) <=         u(3)       <= max thrust (m/s^2)
        %     dist vmin (m/s)    <= [d(1); d(3); d(5)] <= dist vmax (m/s)
        %     dist amin (m/s^2)  <= [d(7); d(8); d(9)] <= dist amax (m/s^2)

      
        if nargin < 1
            x = zeros(obj.nx, 1);
        end
        
        if nargin < 2
            angleMax = deg2rad(15);
            uMin = [-angleMax; -angleMax; 4];
            uMax = [angleMax; angleMax; 16];
        end
        
        if nargin < 4
            dMin = zeros(1,6);
            dMax = zeros(1,6);
        end
        
        
        if nargin < 6
            dims = [1,4];
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
        obj.nd = 6;
        obj.pdim = [1 2 3];
        obj.vdim = [4 5 6];
    end
  end
end