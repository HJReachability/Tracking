% overall IDEA: first use existing run code and script. Then use Obstacle map
% to run again, keeping already developed tree, etc.

% QUESTION: how can I modify it so that the obstacle sensing is done in the
% beginning without modifying the class? I guess this form would be more
% useful if new obstacles were added in so that the global obstacles were
% also modified, but...

% Can modify the given RrtPlanner Class to use my SenseNewObstacles method
% when sensing. In the end, I'd have to modify some existing code in the
% lower level haha...

classdef ObstacleMap
    properties
        rrt; % the stored RRT
        global_obs = cell(4, 3); % a cell array of all the obstacles on the map.
        local_obs = cell(4, 3); % a cell array of the obstacles that the drone has seen so far.
    end
    
    methods
   
        % initialization
        function self = ObstacleMap(rrt)
            % input: rrt instance
            if nargin == 1
                % input all the obstacles from the rrt
                % make use of the given GenerateObstacles method
                % TO-DO: format within cell array
                self.rrt = rrt;
                raw_obs = rrt.obs; 
                self.global_obs = raw_obs;
            end
        end
        
        % updates local obstacles as the drone moves
        function UpdateObstacles(rrt)
            SenseNewObstacles(delta_t, rrt);
            % overwrite the original matrix with the new matrix
            self.local_obs = local_obs;
        end
        
        % after moving the drone along the path for a short amount of time
        % delta_t, check surroundings for obstacles. Pass in the
        % in-progress rrt      
       
        % IDEA: sense for obstacles every delta_t seconds 
        % (then this must be modified within the Run method of RRTPlanner)
        function SenseNewObstacles(delta_t, rrt)
            
        end
        
        % Run Again with new seed initial point
        % To be called after the main RRT algorithm has been run. Takes into account the disturbances (hence, new initial points)
         
        function RunAgain(initial_state)
            % modiy self.rrt directly
            if initial_state ~= NULL
                self.rrt.PlantNewSeeds([initial_state]); % plant a seed at the initial state to expand tree
                self.rrt.Connect(self.rrt, initial_state); % connect the rrt from the initial state to the saved rrt
            else % check if initial state has been given. If not, generate new random state.3
                while num_states ~= 0
                    self.rrt.PlantNewSeeds([initial_state]); % plant a seed at the initial state to expand tree
                    self.rrt.Connect(self.rrt, initial_state); % connect the rrt from the initial state to the saved rrt
                end
            end
        end  
        
%         function plot()
%            self.rrt.plot(); % just call existing plot method in RrtPlanner class 
%         end
        
    end
end