classdef ObsVertex < handle
    % Obstacle vertex
    
    properties
        point
        sensed
    end
    
    methods
        %% Constructor.
        function obj = ObsVertex(point)
          obj.point = [point(1) point(2)];
          obj.sensed = false;
        end
    end
    
end