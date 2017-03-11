classdef ObstacleMap < handle
  
  % roughest approx: add entire obstacle in once you've sensed.
  % actual idea: union of all parts of the obstacle you sense. But can't be
  % represented by MATLAB's plane structure.
  
  properties
    rrt; % the stored RRT
    global_obs; % a cell array of all the obstacles on the map.
    local_obs; % a cell array of the obstacles that the drone has seen so far. Starts empty.
    % cube; % the cube sensing range
    point;
    track_err; % the tracking error bound to pad each obstacle with
  end
  
  methods
    % initialization
    % if input point, might not need entire rrt? Find another way of
    % getting global_obs
    function self = ObstacleMap(rrt, point, senseRange, trackErrBnd)
      self.rrt = rrt;
      self.global_obs = rrt.obs;
      self.track_err = trackErrBnd;
      self.local_obs = cell(size(global_obs)); % preallocate memory for local obstacles.
      % Same size as global_obs
      SenseAndUpdate(point, senseRange); % do a first time sensing
    end
    
    % sense for obstacles at the *point* input then update local/known
    % obstacles. use the sensor's sensing *range*. overwrite the original matrix
    % with the new matrix (then this must be modified within the
    % Run method of RRTPlanner)
    
    % First checks if there's any
    % intersection between an obstacle and a plane (a sensor cube's face)
    % and return the two separated parts of the obstacle in an array.
    
    % plane is a 4x3 array. Also updates local obstacles!
    function SenseAndUpdate(obst, point, size)
      % compute the coordinates' ranges of the cube
      [mostNegative, mostPositive] = MakeCubeRange(point, size);
      oneCubeRan = [mostNegative(1), mostPositive(1)];
      twoCubeRan = [mostNegative(2), mostPositive(2)];
      threeCubeRan = [mostNegative(3), mostPositive(3)];
      
      for i = 1:length(obst) % iterate over the indices of the global obst set
        for point = obs % iterate over the four corner points that make up the obs
          % if the point is within the sensing cube...
          if oneCubeRan(1) <= point(1) <= oneCubeRan(2) && twoCubeRan(1) <= point(2) <= twoCubeRan(2) && threeCubeRan(1) <= point(3) <= threeCubeRan(2)
            % there is a point of th obstacle within sensing range. Just
            % add entire obstacle into local
            self.local_obs(i) = self.global_obs(i);
            break
          end
        end
      end
    end
       
    % helper function for SenseAndUpdate. Creates the sensing radius
    % and cube data structure used for sensing
    % point: center of the cube
    % size: length of a side of the cube
    function [smallest, largest] = MakeCubeRange(point, size)
      largest = [point(1) + size/2, point(2) + size/2, point(3) + size/2];
      smallest = [point(1) - size/2, point(2) - size/2, point(3) - size/2];
    end

  end
end