classdef ObstacleMapLS < handle
  % Obstacle map class for level set methods % 
  
  properties
    % 2D Obtacles
    g2D
    global_obs
    local_obs
    padded_obs
    last_sense_region
    
    % handles for plots
    hG
    hL
    hP
    hS
  end
  
  methods
    %% Constructor.
    function obj = ObstacleMapLS(g2D, obs2D)
      obj.g2D = g2D;
      obj.global_obs = obs2D;
      obj.local_obs = inf(g2D.N');
      obj.padded_obs = inf(g2D.N');
    end
    
    %% SenseAndUpdate
    % sense for obstacles that are within sense_range of the point
    % then update local and padded obstacles.
    function new_sensed = sense_update(obj, point, sense_region, track_err)
      % point is 3D
      % sense_region has its own grid and data (subzero level set is the region)
      
      if ~iscolumn(point)
        point = point';
      end
      
      % Migrate sensing region to global grid
      sDataRot = rotateData(sense_region.g, sense_region.data, point(3), ...
        [1 2], []);
      sGridShift = shiftGrid(sense_region.g, point(1:2));
      obj.last_sense_region = migrateGrid(sGridShift, sDataRot, obj.g2D);
      
      % Sense (take intersection of sensing region and local obs)
      obj.last_sense_region = addCRadius(obj.g2D, obj.last_sense_region, 0);
      new_region = max(obj.global_obs, obj.last_sense_region);
      
      % (take union of new_region and local_obs)
      new_local_obs = min(obj.local_obs, new_region);
      
      % Check if new obstacles have been sensed
      if nnz(new_local_obs < 0) > nnz(obj.local_obs < 0)
        new_sensed = true;
        obj.local_obs = new_local_obs;
        obj.padded_obs = addCRadius(obj.g2D, obj.local_obs, track_err);
      else
        new_sensed = false;
      end
      
    end

    %% ObstaclePlot
    function plotGlobal(obj, color, linestyle)
      if nargin < 2
        color = 'k';
      end
      
      if nargin < 3
        linestyle = ':';
      end
      
      % Global obstacles
      if ~isempty(obj.hG)
        delete(obj.hG)
      end
      
      extraArgs.LineStyle = linestyle;
      obj.hG = visSetIm(obj.g2D, obj.global_obs, color, 0, extraArgs);
    end
    
    function plotLocal(obj, color, linestyle)
      if nargin < 2
        color = 'r';
      end
      
      if nargin < 3
        linestyle = '-';
      end
      
      % Local obstacles
      if ~isempty(obj.hL)
        delete(obj.hL)
      end
      
      extraArgs.LineStyle = linestyle;
      obj.hL = visSetIm(obj.g2D, obj.local_obs, color, 0, extraArgs);
    end
    
    function plotPadded(obj, color, linestyle)
      if nargin < 2
        color = 'b';
      end
      
      if nargin < 3
        linestyle = '--';
      end
      
      % Local obstacles
      if ~isempty(obj.hP)
        delete(obj.hP)
      end
      
      extraArgs.LineStyle = linestyle;
      obj.hP = visSetIm(obj.g2D, obj.padded_obs, color, 0, extraArgs);
    end
    
    function plotSenseRegion(obj)
      if nargin < 2
        color = 'g';
      end
      
      if nargin < 3
        linestyle = '-';
      end
      
      % Local obstacles
      if ~isempty(obj.hS)
        delete(obj.hS)
      end
      
      extraArgs.LineStyle = linestyle;
      obj.hS = visSetIm(obj.g2D, obj.last_sense_region, color, 0, extraArgs);      
    end
  % END OF METHODS
  end
end