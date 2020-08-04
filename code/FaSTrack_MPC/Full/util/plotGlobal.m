function h = plotGlobal(global_obs, color, linestyle, extraArgs)
% Plot global obstacles

    %% Default
    if nargin < 3
      color = 'k';
    end

    if nargin < 4
      linestyle = ':';
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
    nOb = length(global_obs);

    % (TODO) Flexible size
    [pll,plr,pur,pul] = findVertices(global_obs{1});
    obs1 = [pll;plr;pur;pul];
    [pll,plr,pur,pul] = findVertices(global_obs{2});
    obs2 = [pll;plr;pur;pul];
    [pll,plr,pur,pul] = findVertices(global_obs{3});
    obs3 = [pll;plr;pur;pul];

    lOb = getlOb(obs1,obs2,obs3);
    vOb = [4,4,4]; %TODO

    h = [];

    % Plot Obstacle
    for i = 1 : nOb
        for j = 1 : vOb(i)
            h = [h line([lOb{i,j}(1),lOb{i,j+1}(1)] , [lOb{i,j}(2),lOb{i,j+1}(2)],'color',color,'LineStyle',linestyle)];
            hold on
        end
    end

    if drawPoints
        for i = 1:nOb
            for vertex = global_obs{i}
                h = [h plot(vertex.point(1),vertex.point(2),'.','color',color)];
                hold on
            end
        end
    end


    % xlabel('x', 'FontSize', 16)
    % ylabel('y', 'FontSize', 16)
    xlabel('x')
    ylabel('y')

    axis equal
    xlim([-2,14.0])
    ylim([-2,14.0])

    box on
    grid on

    drawnow

end