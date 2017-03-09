classdef ObstacleMap < handle
    properties
        rrt; % the stored RRT
        global_obs; % a cell array of all the obstacles on the map.
        local_obs; % a cell array of the obstacles that the drone has seen so far. Starts empty.
        cube; % the cube sensing range
        track_err; % the tracking error bound to pad each obstacle with
    end
    
    methods
        % initialization
        % if input point, might not need entire rrt? Find another way of
        % getting global_obs
        function self = ObstacleMap(rrt, point, senseRange, trackErrBnd)
            self.rrt = rrt;
            if nargin == 4
                self.global_obs = rrt.obs;
                self.cube = MakeCubeRange(point, senseRange);
                self.track_err = trackErrBnd; 
                
                glbSize = size(global_obs);
                self.local_obs = cell(4, 3, glbSize(3)); % preallocate memory for local obstacles. 
                                                         % Same size as global_obs
            end
        end
        
        % sense for obstacles at the *point* input then update local/known 
        % obstacles. use the sensor's sensing *range*. overwrite the original matrix 
        % with the new matrix (then this must be modified within the 
        % Run method of RRTPlanner)
        
        function SenseAndUpdate(point, range)
            % sense
            senseRange = MakeCubeRange(point, range);
            % all the obstacles that have not been explored yet
            other_obs = setdiff(self.global_obs, self.local_obs);
            if ~isempty(other_obs) % if there are still some other unexplored obstacles...
                % check for obstacle intersection with each face that makes up
                % the sensing range
                for face = senseRange
                    for obst = other_obs
                        obs = CheckIntersection(obst, face); % just use global_obs and index the same way as 
                                                      % local_obs so update part into whole obs is easier...
                    end
                end
            end
        end
        
        % helper function for SenseAndUpdate. Creates the sensing radius
        % and cube data structure used for sensing
        % point: center of the cube
        % size: length of a side of the cube
        function cube = MakeCubeRange(point, size)
            cube = [];
            cube(:,1) = [point(1) + size/2, point(2) + size/2, point(3) + size/2];
            cube(:,2) = [point(1) + size/2, point(2) - size/2, point(3) + size/2];
            cube(:,3) = [point(1) + size/2, point(2) + size/2, point(3) - size/2];
            cube(:,4) = [point(1) + size/2, point(2) - size/2, point(3) - size/2];
            cube(:,5) = [point(1) - size/2, point(2) + size/2, point(3) + size/2];
            cube(:,6) = [point(1) - size/2, point(2) - size/2, point(3) + size/2];
            cube(:,7) = [point(1) - size/2, point(2) + size/2, point(3) - size/2];
            cube(:,8) = [point(1) - size/2, point(2) - size/2, point(3) - size/2];
        end
        
        % helper function for SenseAndUpdate. Checks if there's any
        % intersection between an obstacle and a plane (a sensor cube's face)
        % and return the two separated parts of the obstacle in an array.
        
        % Plane is a set of coordinates such that when viewed from inside the cube, the points 
        % are in a counterclockwise orientation?
        % Then normal vector N points inwards and check sign of dot product with
        % point P on cube face and Q on obs 1 or 2. v = dot(Q - P, N);
        
        % plane is a 4x3 array. Also updates local obstacles!
        function CheckIntersection(obst, cube)
            % compute the coordinates' ranges of the cube
            mostPositive = cube(:,1);
            mostNegative = cube(:,8);
            oneCubeRan = [mostNegative(1), mostPositive(1)];
            twoCubeRan = [mostNegative(2), mostPositive(2)];
            threeCubeRan = [mostNegative(3), mostPositive(3)];
            
            for obs = obst % iterate over the global obst set
                numInRange = 0; % count the number of corners of the plane that are within the cube
                for point = obs % iterate over the four corner points that make up the obs
                    % if the point is within the sensing cube...
                    if oneCubeRan(1) <= point(1) <= oneCubeRan(2) && twoCubeRan(1) <= point(2) <= twoCubeRan(2) && threeCubeRan(1) <= point(3) <= threeCubeRan(2) 
                        numInRange = numInRange + 1;
                    end
                    
                    if numInRange == 3
                        % update local obs to combine entire obs// use obst
                        % index so that smae index coud be used for local
                        % obs
                        % pad the obs with track err
                        % break the loop early
                    end
                end
                if numInRange == 2
                    % cut off the local obs at sense range. Do the brute
                    % force checking
                end
                % having only one corner in the cube is impossible with
                % this setup
            end
        end
        
    end
end