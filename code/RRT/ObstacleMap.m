classdef ObstacleMap < handle  

  properties
    global_obs; % a three dimensional matrix of all the obstacles on the map. TODO: reformatting into cell etc...
    local_obs; % a three dimensional matrix of the obstacles that the drone has seen so far. Starts empty.
    padded_obs; % three dimensional matrix of the obstacles that the drone has seen so far PLUS trackErrBd
    num_obs; % the number of global obstacles
    seen_obs; % 1 x num_obs vector of obstacles seen so far, so that it isn't added again into local_obs
    indexx = 0; % counter for number of local obstacles added in
  end
  
  methods
    %% Constructor.
    function self = ObstacleMap(obs)
        self.num_obs = size(obs, 3);
        self.global_obs = obs;
        self.local_obs = inf(size(self.global_obs));
        self.padded_obs = inf(4, 3, 6.*self.num_obs);
        self.seen_obs = zeros(self.num_obs, 1);
    end
    
    %% SenseAndUpdate
    % sense for obstacles that are within sense_range of the point
    % then update local and padded obstacles.
    function SenseAndUpdate(self, point, sense_range, track_err)
      % compute the cube sensing range where point is the center 
      % and sense_range is the side length
      cubeMin = point - sense_range;
      cubeMax = point + sense_range;
      
      for i = 1:self.num_obs
        obstacle = self.global_obs(:, :, i); % iterate over global obs
        % check if obstacle has been seen before
        if ~self.seen_obs(i)         
          for j = 1:size(obstacle, 1) % iterate over the four corner points that make up the obs
            % check if point is within the sensing cube 
            if self.AbleToSense(cubeMin, cubeMax, obstacle)
              self.seen_obs(i) = 1; % mark as seen
              self.TrackErrorPadding(track_err, obstacle); % pad it and add it into the padded set
              self.local_obs(:, :, self.indexx./6) = obstacle; % add to local set
              break;
            end
         end
       end
     end
    end
   
    %% AbleToSense
    % helper function for SenseAndUpdate that determines if obstacle can
    % be sensed or not
    
    function value = AbleToSense(self, cubeMin, cubeMax, obstacle)
      % compute the max and min coordinates of obstacs
      one = unique(obstacle(:, 1));
      two = unique(obstacle(:, 2));
      three = unique(obstacle(:, 3));
          
      value = (((cubeMin(1) <= min(one)) && (min(one) <= cubeMax(1))) || ((cubeMin(1) <= max(one)) && (max(one) <= cubeMax(1)))) && ...
        (((cubeMin(2) <= min(two)) && (min(two) <= cubeMax(2))) || ((cubeMin(2) <= max(two)) && (max(two) <= cubeMax(2)))) && ...
        (((cubeMin(3) <= min(three)) && (min(three) <= cubeMax(3))) || ((cubeMin(3) <= max(three)) && (max(three) <= cubeMax(3))));
    end
    
    %% TrackErrorPadding
    % helper function for SenseAndUpdate that adds the tracking error
    % bound to each obstacle before adding it to padded
    function TrackErrorPadding(self, err, obs)
        % assume all obstacles are axes-aligned
        % find the coordinate that remains constant
        one = unique(obs(:, 1));
        two = unique(obs(:, 2));        
        if length(one) == 1
          index = 1;
        elseif length(two) == 1
          index = 2;
        else
          index = 3;
        end
        % now the constant coordinate is *index*
        
        % IMPORTANT: required obstacle file format: start with lowermost/least-coordinate 
        % point then go clockwise from there (up, right, down, back to start)
        
        pointOne = obs(1, :);
        pointTwo = obs(2, :);
        pointThree = obs(3, :);
        pointFour = obs(4, :);

        % front and back faces extended
        new_obs_front = cell(4, 1);
        new_obs_back = cell(4, 1);
        
        if index == 2 % if obstacle is flat on y-plane
          new_obs_front{1} = [pointOne(1) - err, pointOne(2) + err, pointOne(3) - err];
          new_obs_front{2} = [pointTwo(1) - err, pointTwo(2) + err, pointTwo(3) + err];
          new_obs_front{3} = [pointThree(1) + err, pointThree(2) + err, pointThree(3) + err];
          new_obs_front{4} = [pointFour(1) + err, pointFour(2) + err, pointFour(3) - err];
          
          self.indexx = self.indexx + 1;
          self.padded_obs(:,:, self.indexx) = cell2mat(new_obs_front);

          new_obs_back{1} = [pointOne(1) - err, pointOne(2) - err, pointOne(3) - err];
          new_obs_back{2} = [pointTwo(1) - err, pointTwo(2) - err, pointTwo(3) + err];
          new_obs_back{3} = [pointThree(1) + err, pointThree(2) - err, pointThree(3) + err];
          new_obs_back{4} = [pointFour(1) + err, pointFour(2) - err, pointFour(3) - err];
          self.indexx = self.indexx + 1;
          self.padded_obs(:,:,self.indexx) = cell2mat(new_obs_back);
          
        elseif index == 3 % flat on z-plane
          new_obs_front{1} = [pointOne(1) - err, pointOne(2) - err, pointOne(3) + err];
          new_obs_front{2} = [pointTwo(1) - err, pointTwo(2) + err, pointTwo(3) + err];
          new_obs_front{3} = [pointThree(1) + err, pointThree(2) + err, pointThree(3) + err];
          new_obs_front{4} = [pointFour(1) + err, pointFour(2) - err, pointFour(3) + err];
          
          self.indexx = self.indexx + 1;
          self.padded_obs(:,:, self.indexx) = cell2mat(new_obs_front);

          new_obs_back{1} = [pointOne(1) - err, pointOne(2) - err, pointOne(3) - err];
          new_obs_back{2} = [pointTwo(1) - err, pointTwo(2) + err, pointTwo(3) - err];
          new_obs_back{3} = [pointThree(1) + err, pointThree(2) + err, pointThree(3) - err];
          new_obs_back{4} = [pointFour(1) + err, pointFour(2) - err, pointFour(3) - err];
          self.indexx = self.indexx + 1;
          self.padded_obs(:,:,self.indexx) = cell2mat(new_obs_back);
          
        else % flat on x-plane
          new_obs_front{1} = [pointOne(1) + err, pointOne(2) - err, pointOne(3) - err];
          new_obs_front{2} = [pointTwo(1) + err, pointTwo(2) - err, pointTwo(3) + err];
          new_obs_front{3} = [pointThree(1) + err, pointThree(2) + err, pointThree(3) + err];
          new_obs_front{4} = [pointFour(1) + err, pointFour(2) + err, pointFour(3) - err];
          
          self.indexx = self.indexx + 1;
          self.padded_obs(:,:, self.indexx) = cell2mat(new_obs_front);

          new_obs_back{1} = [pointOne(1) - err, pointOne(2) - err, pointOne(3) - err];
          new_obs_back{2} = [pointTwo(1) - err, pointTwo(2) - err, pointTwo(3) + err];
          new_obs_back{3} = [pointThree(1) - err, pointThree(2) + err, pointThree(3) + err];
          new_obs_back{4} = [pointFour(1) - err, pointFour(2) + err, pointFour(3) - err];
          self.indexx = self.indexx + 1;
          self.padded_obs(:,:,self.indexx) = cell2mat(new_obs_back);          
        end
        
        % four edge/border faces to close off
        new_obs_side = cell(4, 1);
        
        new_obs_side{1} = new_obs_back{1};
        new_obs_side{2} = new_obs_back{2};
        new_obs_side{3} = new_obs_front{2};
        new_obs_side{4} = new_obs_front{1};
        self.indexx = self.indexx + 1;
        self.padded_obs(:,:,self.indexx) = cell2mat(new_obs_side); 
        
        new_obs_side{1} = new_obs_back{2};
        new_obs_side{2} = new_obs_back{3};
        new_obs_side{3} = new_obs_front{3};
        new_obs_side{4} = new_obs_front{2};
        self.indexx = self.indexx + 1;
        self.padded_obs(:,:,self.indexx) = cell2mat(new_obs_side); 
        
        new_obs_side{1} = new_obs_back{4};
        new_obs_side{2} = new_obs_back{3};
        new_obs_side{3} = new_obs_front{3};
        new_obs_side{4} = new_obs_front{4};
        self.indexx = self.indexx + 1;
        self.padded_obs(:,:,self.indexx) = cell2mat(new_obs_side); 
        
        new_obs_side{1} = new_obs_back{1};
        new_obs_side{2} = new_obs_back{4};
        new_obs_side{3} = new_obs_front{4};
        new_obs_side{4} = new_obs_front{1};
        self.indexx = self.indexx + 1;
        self.padded_obs(:,:,self.indexx) = cell2mat(new_obs_side);
    end

    %% ObstaclePlot
    % plots all the obstacles, returns the handles for local, global, etc.
    
    function [hG, hL, hP] = ObstaclePlot(self, plotGlobal, plotLocal, plotPadded)
      % plot with global obstacles
      if plotGlobal
        if size(self.global_obs,1)>0
          for i = 1:size(self.global_obs,3)
            hG(i) = fill3([self.global_obs(1,1,i) self.global_obs(2,1,i) self.global_obs(3,1,i) self.global_obs(4,1,i) self.global_obs(1,1,i)] ...
              ,[self.global_obs(1,2,i) self.global_obs(2,2,i) self.global_obs(3,2,i) self.global_obs(4,2,i) self.global_obs(1,2,i)] ...
              ,[self.global_obs(1,3,i) self.global_obs(2,3,i) self.global_obs(3,3,i) self.global_obs(4,3,i) self.global_obs(1,3,i)] ...
              ,'b','EdgeAlpha',0);
            alpha(0.1);
            if i == 1
              hold on;
            end
          end
        end
      end

      % plot with local obstacles
      if plotLocal
        if size(self.local_obs,1)>0
          for i = 1:size(self.local_obs,3)
            hL(i) = fill3([self.local_obs(1,1,i) self.local_obs(2,1,i) self.local_obs(3,1,i) self.local_obs(4,1,i) self.local_obs(1,1,i)] ...
              ,[self.local_obs(1,2,i) self.local_obs(2,2,i) self.local_obs(3,2,i) self.local_obs(4,2,i) self.local_obs(1,2,i)] ...
              ,[self.local_obs(1,3,i) self.local_obs(2,3,i) self.local_obs(3,3,i) self.local_obs(4,3,i) self.local_obs(1,3,i)] ...
              ,'r','EdgeAlpha',0);
            alpha(0.1);
            if i == 1
              hold on;
            end
          end
        end
      end

      % plot with padded obstacles
      if plotPadded
        if size(self.padded_obs,1)>0
          for i = 1:size(self.padded_obs,3)
            hP(i) = fill3([self.padded_obs(1,1,i) self.padded_obs(2,1,i) self.padded_obs(3,1,i) self.padded_obs(4,1,i) self.padded_obs(1,1,i)] ...
              ,[self.padded_obs(1,2,i) self.padded_obs(2,2,i) self.padded_obs(3,2,i) self.padded_obs(4,2,i) self.padded_obs(1,2,i)] ...
              ,[self.padded_obs(1,3,i) self.padded_obs(2,3,i) self.padded_obs(3,3,i) self.padded_obs(4,3,i) self.padded_obs(1,3,i)] ...
              ,'g','EdgeAlpha',0);
            alpha(0.1);
            if i == 1
              hold on;
            end
          end
        end
      end
    end
    
  end
end