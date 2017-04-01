classdef ObstacleMap < handle
  
  properties
    global_obs; % a three dimensional matrix of all the obstacles on the map. TODO: reformatting into cell etc...
    local_obs; % a three dimensional matrix of the obstacles that the drone has seen so far. Starts empty.
    padded_obs; % three dimensional matrix of the obstacles that the drone has seen so far PLUS trackErrBd
    num_obs; % the number of global obstacles
    seen_obs; % 1 x num_obs vector of obstacles seen so far, so that it isn't added again into local_obs
    indexx = 0; % counter for number of local obstacles added in
    
    % handle for the global obstacle plot
    hG
    
    % same for local obs
    hL
    
    % same for padded obs
    hP
    
  end
  
  methods
    %% Constructor.
    function self = ObstacleMap(obs)
      self.num_obs = size(obs, 3);
      self.global_obs = obs;
      self.local_obs = inf(size(self.global_obs));
      self.padded_obs = inf(4, 3, 6.*self.num_obs);
      self.seen_obs = false(1, self.num_obs);
    end
    
    %% SenseAndUpdate
    % sense for obstacles that are within sense_range of the point
    % then update local and padded obstacles.
    function new_sensed = sense_update(obj, point, sense_range, track_err)
      % SenseAndUpdate(obj, point, sense_range, track_err)
      if ~iscolumn(point)
        point = point';
      end
      
      sense_ranges = [point-sense_range, point+sense_range];
      new_sensed = false;
      
      for i = find(~obj.seen_obs) % iterate over unseen global obs
        if obj.ableToSense(sense_ranges, i)
          obj.seen_obs(i) = true; % mark as seen
          
          % pad it and add it into the padded set
          obj.pad(track_err, obj.global_obs(:,:,i));
          
          % add to local set
%           small = 1e-3;
          obj.local_obs(:, :, obj.indexx./6) = obj.global_obs(:,:,i)-1e-2;
          
          new_sensed = true;
        end
      end
    end
    
    %% AbleToSense
    % helper function for SenseAndUpdate that determines if obstacle can
    % be sensed or not
    function contained = chk_containment(obj, interval1, interval2)
      % function contained = chk_containment(interval1, interval2)
      if length(interval1) ~= 2 || length(interval2) ~= 2
        error('Intervals must be of length 2!')
      end
      
      % Determine which interval is smaller
      if abs(diff(interval1)) < abs(diff(interval2))
        small = interval1;
        big = interval2;
      else
        small = interval2;
        big = interval1;
      end
      
      for i = 1:length(small)
        if small(i) >= min(big) && small(i) <= max(big)
          contained = true;
          return
        end
      end
      
      contained = false;
    end
    
    function sensed = ableToSense(obj, sense_ranges, obs_ind)
      % compute the max and min coordinates of obstacs
      
      obstacle = obj.global_obs(:,:,obs_ind);
      for i = 1:3
        obs_range_i = [min(obstacle(:,i)) max(obstacle(:,i))];
        
        if ~obj.chk_containment(sense_ranges(i,:), obs_range_i)
          sensed = false;
          return
        end
      end
      
      sensed = true;
    end
    
    %% TrackErrorPadding
    % helper function for SenseAndUpdate that adds the tracking error
    % bound to each obstacle before adding it to padded
    function pad(self, err, obs)
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

    
    function plotGlobal(obj, color, linestyle)
      if nargin < 2
        color = 'k';
      end
      
      if nargin < 3
        linestyle = 'none';
      end
      
      % Global obstacles
      coords = get_obs_coords_for_plot(obj.global_obs);
      
      if ~isempty(obj.hG)
        delete(obj.hG)
      end
      
      obj.hG = fill3(coords{:}, color, 'FaceAlpha', 0.1, 'LineStyle',linestyle);
    end
    
    function plotLocal(obj, color)
      if nargin < 2
        color = 'r';
      end
      
      % Local obstacles
      coords = get_obs_coords_for_plot(obj.local_obs);
      
      if ~isempty(obj.hL)
        delete(obj.hL)
      end
      
      obj.hL = fill3(coords{:}, color, 'FaceAlpha', 0.7, 'LineStyle', 'none');
    end
    
    function plotPadded(obj)
      if nargin < 2
        color = 'g';
      end
      
      % Augmented obstacles
      coords = get_obs_coords_for_plot(obj.padded_obs);
      
      if ~isempty(obj.hP)
        delete(obj.hP)
      end
      
      obj.hP = fill3(coords{:}, color, 'FaceAlpha', 0.05, 'LineStyle', 'none');
    end
     
  % END OF METHODS
  end
end