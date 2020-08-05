function h = plotLocal(local_obs, color, linestyle, extraArgs)
% Plot currently seen obstacles

    %% Default
    if nargin < 3
      color = 'r';
    end

    if nargin < 4
      linestyle = '-';
    end

    if nargin < 5
      extraArgs = [];
    end

    drawPoints = false;
    if isfield(extraArgs, 'drawPoints')
      drawPoints = extraArgs.drawPoints;
    end

    save_png = false;
    if isfield(extraArgs, 'fig_filename')
      save_png = true;
      fig_filename = extraArgs.fig_filename;
    end

    % Number of individual obstacles
    nOb = length(local_obs);

    % (TODO) Flexible size
    if length(local_obs{1}) > 3
        [pll,plr,pur,pul] = findVertices(local_obs{1});
        obs1 = refineVertex([pll;plr;pur;pul]);
    else
        obs1 = [NaN NaN; NaN NaN; NaN NaN; NaN NaN];
    end

    if length(local_obs{2}) > 3
        [pll,plr,pur,pul] = findVertices(local_obs{2});
        obs2 = refineVertex([pll;plr;pur;pul]);
    else
        obs2 = [NaN NaN; NaN NaN; NaN NaN; NaN NaN];
    end

    if length(local_obs{3}) > 3
        [pll,plr,pur,pul] = findVertices(local_obs{3});
        obs3 = refineVertex([pll;plr;pur;pul]);
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
                h = [h line([lOb{i,j}(1),lOb{i,j+1}(1)] , [lOb{i,j}(2),lOb{i,j+1}(2)],'color',color,'LineStyle',linestyle)];            
            end
        end
    end

    if drawPoints
        for i = 1:nOb
            for vertex = local_obs{i}
                h = [h plot(vertex.point(1),vertex.point(2),'.','color',color)];
                hold on
            end
        end
    end

    drawnow

end