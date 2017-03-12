classdef ObstacleMap < handle  
  % TODO: plot time shots of local obst updating to eventually get to
  % global obst, understand where environment walls are
  
  properties
    global_obs; % a matrix of all the obstacles on the map. TODO: reformatting into cell etc...
    local_obs; % a three dimensional matrix of the obstacles that the drone has seen so far. Starts empty.
  end
  
  methods
    % constructor. Calls for the sense and update method immediately.
    function self = ObstacleMap(obs, point, senseRange, trackErrBnd)
      if ~self.global_obs % so that we won't have to overwrite global obstacles.
        self.global_obs = obs;
      end
      
      % self.local_obs = cell(4, 3);
      SenseAndUpdate(self.global_obs, point, senseRange, trackErrBnd);
    end
    
    % sense for obstacles that are within the sensing range (*size*) at the *point* given
    % then update local/known obstacles. overwrite the original matrix
    % with the new matrix    
    function SenseAndUpdate(obst, point, size, track_err)
      % compute the coordinates' ranges of the cube sensing range
      [mostNegative, mostPositive] = MakeCubeRange(point, size);
      
      oneCubeRan = [mostNegative(1), mostPositive(1)];
      twoCubeRan = [mostNegative(2), mostPositive(2)];
      threeCubeRan = [mostNegative(3), mostPositive(3)];
      
      for i = 1:length(obst) % iterate over the indices of the (self.global) obst set
        for point = obst(i) % iterate over the four corner points that make up the obs
          % if the point is within the sensing cube...
          if oneCubeRan(1) <= point(1) <= oneCubeRan(2) && twoCubeRan(1) <= point(2) <= twoCubeRan(2) && threeCubeRan(1) <= point(3) <= threeCubeRan(2)
            obs = obst(i);
            % pad it and add it into the local set
            TrackErrorPadding(track_err, obs);
            break
          end
        end
      end
    end
       
    % helper function for SenseAndUpdate. Creates the cube structure used for sensing
    % point: center of the cube
    % size: length of a side of the cube
    function [smallest, largest] = MakeCubeRange(point, size)
      largest = [point(1) + size/2, point(2) + size/2, point(3) + size/2];
      smallest = [point(1) - size/2, point(2) - size/2, point(3) - size/2];
    end
    
    % helper function for sense and update that adds the tracking error
    % bound to each obstacle before adding from global to local
    % don't use for loop?
    function TrackErrorPadding(err, obs)
        % find the coordinate that remains constant
        countOne = 0;
        countTwo = 0;
        countThree = 0;
        for i = 1:3
          pt = obs(i);
          ptTwo = obs(i + 1);
          if pt(1) == ptTwo(1)
            countOne = countOne + 1;
          end
          if pt(2) == ptTwo(2)
            countTwo = countTwo + 1;
          end
          if pt(3) == ptTwo(3)
            countThree = countThree + 1;
          end
        end
        
        if countOne == 3
          index = 1;
        elseif countTwo == 3
          index = 2;
        else
          index = 3;
        end
        % now the constant coordinate is *index*
        
        % required obstacle file format: start with lowermost/least-coordinate 
        % point then go clockwise from there (up, right, down, back to start)
        
        if index == 2 % if obstacle is flat on y-plane
          % front face extended
          new_obs_front = cell(4, 1);
          point = obs{1};
          new_obs_front(1) = [point(1) - err, point(2) + err, point(3) - err];
          point = obs{2};
          new_obs_front(2, 1) = [point(1) - err, point(2) + err, point(3) + err];
          point = obs(3);
          new_obs_front{3} = [point(1) + err, point(2) + err, point(3) + err];
          point = obs(4);
          new_obs_front{4} = [point(1) + err, point(2) + err, point(3) - err];
          
          if ~self.local_obs % if no obstacles have been sensed yet
            self.local_obs = cell2mat(new_obs_front);          
          else
            self.local_obs(:,:,length(self.local_obs) + 1) = cell2mat(new_obs_front);
          end

          % back face extended
          new_obs_back = cell(4, 1);
          point = obs(1);
          new_obs_back{1} = [point(1) - err, point(2) - err, point(3) - err];
          point = obs(2);
          new_obs_back{2} = [point(1) - err, point(2) - err, point(3) + err];
          point = obs(3);
          new_obs_back{3} = [point(1) + err, point(2) - err, point(3) + err];
          point = obs(4);
          new_obs_back{4} = [point(1) + err, point(2) - err, point(3) - err];
          self.local_obs(:,:,length(self.local_obs) + 1) = cell2mat(new_obs_back);
          
        elseif index == 3 % flat on z-plane
          new_obs_front = cell(4, 1); 
          point = obs(1);
          new_obs_front{1} = [point(1) - err, point(2) - err, point(3) + err];
          point = obs(2);
          new_obs_front{2} = [point(1) - err, point(2) + err, point(3) + err];
          point = obs(3);
          new_obs_front{3} = [point(1) + err, point(2) + err, point(3) + err];
          point = obs(4);
          new_obs_front{4} = [point(1) + err, point(2) - err, point(3) + err];
          
          if ~self.local_obs
            self.local_obs = cell2mat(new_obs_front);          
          else
            self.local_obs(:,:,length(self.local_obs) + 1) = cell2mat(new_obs_front);
          end

          new_obs_back = cell(4, 1);
          point = obs(1);
          new_obs_back{1} = [point(1) - err, point(2) - err, point(3) - err];
          point = obs(2);
          new_obs_back{2} = [point(1) - err, point(2) + err, point(3) - err];
          point = obs(3);
          new_obs_back{3} = [point(1) + err, point(2) + err, point(3) - err];
          point = obs(4);
          new_obs_back{4} = [point(1) + err, point(2) - err, point(3) - err];
          self.local_obs(:,:,length(self.local_obs) + 1) = cell2mat(new_obs_back);
          
        else % flat on x-plane
          new_obs_front = cell(4, 1);
          point = obs(1);
          new_obs_front{1} = [point(1) + err, point(2) - err, point(3) - err];
          point = obs(2);
          new_obs_front{2} = [point(1) + err, point(2) - err, point(3) + err];
          point = obs(3);
          new_obs_front{3} = [point(1) + err, point(2) + err, point(3) + err];
          point = obs(4);
          new_obs_front{4} = [point(1) + err, point(2) + err, point(3) - err];
          
          if ~self.local_obs
            self.local_obs = cell2mat(new_obs_front);          
          else
            self.local_obs(:,:,length(self.local_obs) + 1) = cell2mat(new_obs_front);
          end

          new_obs_back = cell(4, 1);
          point = obs(1);
          new_obs_back{1} = [point(1) - err, point(2) - err, point(3) - err];
          point = obs(2);
          new_obs_back{2} = [point(1) - err, point(2) - err, point(3) + err];
          point = obs(3);
          new_obs_back{3} = [point(1) - err, point(2) + err, point(3) + err];
          point = obs(4);
          new_obs_back{4} = [point(1) - err, point(2) + err, point(3) - err];
          self.local_obs(:,:,length(self.local_obs) + 1) = cell2mat(new_obs_back);          
        end
        
        % four edge/borders to close off
        new_obs_side = cell(4, 1);
        
        new_obs_side{1} = new_obs_back{1};
        new_obs_side{2} = new_obs_back{2};
        new_obs_side{3} = new_obs_front{2};
        new_obs_side{4} = new_obs_front{1};
        self.local_obs(:,:,length(self.local_obs) + 1) = cell2mat(new_obs_side); 
        
        new_obs_side{1} = new_obs_back{2};
        new_obs_side{2} = new_obs_back{3};
        new_obs_side{3} = new_obs_front{3};
        new_obs_side{4} = new_obs_front{2};
        self.local_obs(:,:,length(self.local_obs) + 1) = cell2mat(new_obs_side); 
        
        new_obs_side{1} = new_obs_back{4};
        new_obs_side{2} = new_obs_back{3};
        new_obs_side{3} = new_obs_front{3};
        new_obs_side{4} = new_obs_front{4};
        self.local_obs(:,:,length(self.local_obs) + 1) = cell2mat(new_obs_side); 
        
        new_obs_side{1} = new_obs_back{1};
        new_obs_side{2} = new_obs_back{4};
        new_obs_side{3} = new_obs_front{4};
        new_obs_side{4} = new_obs_front{1};
        self.local_obs(:,:,length(self.local_obs) + 1) = cell2mat(new_obs_side); 
    end
    
  end
end