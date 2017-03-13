classdef ObstacleMap < handle  

  properties
    global_obs; % a three dimensional matrix of all the obstacles on the map. TODO: reformatting into cell etc...
    local_obs; % a three dimensional matrix of the obstacles that the drone has seen so far. Starts empty.
    num_obs; % the number of global obstacles
    seen_obs; % obstacles seen so far, so that it isn't added again into local_obs
  end
  
  methods
    %% Constructor. Calls for the sense and update method immediately.
    function self = ObstacleMap(obs)
        self.global_obs = obs;
        dimen = size(obs);
        self.num_obs = dimen(3);
        self.global_obs = mat2cell(self.global_obs, [1 1 1 1], 3, ones(1, self.num_obs)); 
        % self.global_obs is now a cell array
    end
    
    %% SenseAndUpdate
    % sense for obstacles that are within the sensing range (*size*) at the *point* given
    % then update local/known obstacles. overwrite the original matrix
    % with the new matrix    
    function SenseAndUpdate(self, point, size, track_err)
      % compute the cube sensing range where point is the center 
      % and size is the side length
      oneCubeRan = [point(1) - size, point(1) + size];
      twoCubeRan = [point(2) - size, point(2) + size];
      threeCubeRan = [point(3) - size, point(3) + size];
      
      for i = 1:self.num_obs % iterate over the indices of the (self.global) obst set
        [one, two, three, four] = self.global_obs{:, :, i};
        all_points = [one; two; three; four]'; % reformat into regular 4x3 matrix
        for pt = all_points % iterate over the four corner points that make up the obs
          % check if point is within the sensing cube
          if oneCubeRan(1) <= pt(1) <= oneCubeRan(2) && twoCubeRan(1) <= pt(2) <= twoCubeRan(2) && threeCubeRan(1) <= pt(3) <= threeCubeRan(2)
            % pad it and add it into the local set
            disp('obstacle is within range')
            seen = self.ObsAlreadySeen(all_points);
            if ~seen
              all_points = all_points';
              self.TrackErrorPadding(track_err, mat2cell(all_points, [1 1 1 1], 3)); % reformat into cell array of points
            end
            break
          end
        end
      end
    end
    
    %% ObsAlreadySeen
    % brute force way of checking whether a sensed obstacle is already in
    % self.local_obs or not. So no redundancy in local_obs.
    function seen = ObsAlreadySeen(self, all_points)
      seen = false;
      if ~isempty(self.seen_obs)
        for point = all_points
          for pt = self.seen_obs
            if isequal(point, pt)
              disp('seen is true')
              seen = true;
              break;
            end
          end
        end
      end
    end
    
    %% TrackErrorPadding
    % helper function for sense and update that adds the tracking error
    % bound to each obstacle before adding from global to local
    function TrackErrorPadding(self, err, obs)
        % assume all obstacles are axes-aligned
        % find the coordinate that remains constant
        countOne = 0;
        countTwo = 0;
        countThree = 0;
        for i = 1:3
          pt = obs{i};
          ptTwo = obs{i + 1};
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
        pointOne = obs{1};
        pointTwo = obs{2};
        pointThree = obs{3};
        pointFour = obs{4};

        % front and back faces extended
        new_obs_front = cell(4, 1);
        new_obs_back = cell(4, 1);
        
        if index == 2 % if obstacle is flat on y-plane
          new_obs_front{1} = [pointOne(1) - err, pointOne(2) + err, pointOne(3) - err];
          new_obs_front{2} = [pointTwo(1) - err, pointTwo(2) + err, pointTwo(3) + err];
          new_obs_front{3} = [pointThree(1) + err, pointThree(2) + err, pointThree(3) + err];
          new_obs_front{4} = [pointFour(1) + err, pointFour(2) + err, pointFour(3) - err];
          
          if isempty(self.local_obs) % if no obstacles have been sensed yet
            disp('flat on y plane. not sense yet.')
            self.local_obs = cell2mat(new_obs_front);          
          else
            self.local_obs(:,:,length(self.local_obs) + 1) = cell2mat(new_obs_front);
          end

          new_obs_back{1} = [pointOne(1) - err, pointOne(2) - err, pointOne(3) - err];
          new_obs_back{2} = [pointTwo(1) - err, pointTwo(2) - err, pointTwo(3) + err];
          new_obs_back{3} = [pointThree(1) + err, pointThree(2) - err, pointThree(3) + err];
          new_obs_back{4} = [pointFour(1) + err, pointFour(2) - err, pointFour(3) - err];
          self.local_obs(:,:,length(self.local_obs) + 1) = cell2mat(new_obs_back);
          
        elseif index == 3 % flat on z-plane
          new_obs_front{1} = [pointOne(1) - err, pointOne(2) - err, pointOne(3) + err];
          new_obs_front{2} = [pointTwo(1) - err, pointTwo(2) + err, pointTwo(3) + err];
          new_obs_front{3} = [pointThree(1) + err, pointThree(2) + err, pointThree(3) + err];
          new_obs_front{4} = [pointFour(1) + err, pointFour(2) - err, pointFour(3) + err];
          
          if isempty(self.local_obs)
            disp('flat on z plane. not sense yet.')
            self.local_obs = cell2mat(new_obs_front);          
          else
            self.local_obs(:,:,length(self.local_obs) + 1) = cell2mat(new_obs_front);
          end

          new_obs_back{1} = [pointOne(1) - err, pointOne(2) - err, pointOne(3) - err];
          new_obs_back{2} = [pointTwo(1) - err, pointTwo(2) + err, pointTwo(3) - err];
          new_obs_back{3} = [pointThree(1) + err, pointThree(2) + err, pointThree(3) - err];
          new_obs_back{4} = [pointFour(1) + err, pointFour(2) - err, pointFour(3) - err];
          self.local_obs(:,:,length(self.local_obs) + 1) = cell2mat(new_obs_back);
          
        else % flat on x-plane
          new_obs_front{1} = [pointOne(1) + err, pointOne(2) - err, pointOne(3) - err];
          new_obs_front{2} = [pointTwo(1) + err, pointTwo(2) - err, pointTwo(3) + err];
          new_obs_front{3} = [pointThree(1) + err, pointThree(2) + err, pointThree(3) + err];
          new_obs_front{4} = [pointFour(1) + err, pointFour(2) + err, pointFour(3) - err];
          
          if isempty(self.local_obs)
            disp('flat on x plane. not sense yet.')
            self.local_obs = cell2mat(new_obs_front);          
          else
            self.local_obs(:,:,length(self.local_obs) + 1) = cell2mat(new_obs_front);
          end

          new_obs_back{1} = [pointOne(1) - err, pointOne(2) - err, pointOne(3) - err];
          new_obs_back{2} = [pointTwo(1) - err, pointTwo(2) - err, pointTwo(3) + err];
          new_obs_back{3} = [pointThree(1) - err, pointThree(2) + err, pointThree(3) + err];
          new_obs_back{4} = [pointFour(1) - err, pointFour(2) + err, pointFour(3) - err];
          self.local_obs(:,:,length(self.local_obs) + 1) = cell2mat(new_obs_back);          
        end
        
        % four edge/border faces to close off
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
        
        disp('local_obs')
        self.local_obs
    end
    
  end
end