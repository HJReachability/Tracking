function h = plotPadded(padded_obs, color, linestyle, extraArgs)
% Plot obstacles with padded tracking error bound

    %% Default
    if nargin < 3
      color = 'b';
    end

    if nargin < 4
      linestyle = '--';
    end

    if nargin < 5
      extraArgs = [];
    end

    save_png = false;
    if isfield(extraArgs, 'fig_filename')
      save_png = true;
      fig_filename = extraArgs.fig_filename;
    end

    % Number of individual obstacles
    nOb = length(padded_obs);

    % (TODO) Flexible size
    if length(padded_obs{1}) > 3
        obs1 = padded_obs{1};
    else
        obs1 = [NaN NaN; NaN NaN; NaN NaN; NaN NaN];
    end

    if length(padded_obs{2}) > 3
        obs2 = padded_obs{2};
    else
        obs2 = [NaN NaN; NaN NaN; NaN NaN; NaN NaN];
    end

    if length(padded_obs{3}) > 3
        obs3 = padded_obs{3};
    else
        obs3 = [NaN NaN; NaN NaN; NaN NaN; NaN NaN];
    end

    lOb = getlOb(obs1,obs2,obs3);
    vOb = [4,4,4]; %TODO

    h = [];

    % Plot Obstacle
    for i = 1 : nOb
        if isnan(lOb{i,1}(1))
            continue
        else
            for j = 1 : vOb(i)
                h = [h line([lOb{i,j}(1),lOb{i,j+1}(1)] , [lOb{i,j}(2),lOb{i,j+1}(2)],'color',color,'LineStyle',linestyle,'LineWidth',1.5)];            
            end
        end
    end

    drawnow

end