function obs = initObs(vOb,resolution)
% initialize obstacles

    obs = {};
    nobs = length(vOb(:,1));     % Number of individual obstacles = number of lines in vOb
    for i = 1:nobs               % Iterate over obstacles
        x = []; y = [];
        for j = 1:4              % Iterate over four vertices and find min/max value of x and y         
            x = [x vOb{i,j}(1)];
            y = [y vOb{i,j}(2)];                
        end
        xmin = min(x);
        xmax = max(x);
        ymin = min(y);
        ymax = max(y);
        xinterp = xmin:resolution:xmax;
        yinterp = ymin:resolution:ymax;
        obs_t = [];
        for x_t = xinterp
            for y_t = yinterp
                obs_t = [obs_t ObsVertex([x_t,y_t])];
            end
        end
        obs{i} = obs_t;
    end
end