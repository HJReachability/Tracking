classdef ObstacleMap
    properties
        rrt; % the stored RRT
        global_obs; % a cell array of all the obstacles on the map.
        local_obs = cell(4, 3); % a cell array of the obstacles that the drone has seen so far.
    end
    
    methods
        % initialization
        function self = ObstacleMap(rrt)
            if nargin == 1
                self.rrt = rrt;
                self.global_obs = rrt.obs;
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
                        obs1, obs2 = CheckIntersection(obst, face) % just use global_obs and index the same way as 
                                                      % local_obs so update part into whole obs is easier...
                        % check which one of obs1, 2 lies within the cube
                    end
                end
            end
        end
        
        % helper function for SenseAndUpdate. Creates the sensing radius
        % and cube data structure for sensing
        % point will be a vector of xyz coordinates
        function MakeCubeRange(point, range)
            
        end
        
        % helper function for SenseAndUpdate. Checks if there's any
        % intersection between an obstacle and a plane (a sensor cube's face)
        % and return the two separated parts of the obstacle in an array.
        
        % Plane is a set of coordinates such that when viewed from inside the cube, the points are in a clockwise orientation? 
        function CheckIntersection(obst, plane)
            % see CollisionCheck method in RRT...
        end
        
    end
end