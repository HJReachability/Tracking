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
        
        % sense for obstacles at the point input then update local/known 
        % obstacles. overwrite the original matrix with the new matrix
        % (then this must be modified within the Run method of RRTPlanner)
        function SenseAndUpdate(point)
            
        end
        
%         function plot()
%            self.rrt.plot(); % just call existing plot method in RrtPlanner class 
%         end
        
    end
end