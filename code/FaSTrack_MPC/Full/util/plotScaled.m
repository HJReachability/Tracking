function [hSO, hST] = plotScaled(state, lOb, color, linestyle, drawObs)
% Plot scaled obstacles during the iterative warmstart process and the
% associated state trajectory

    %% Defaults
    if nargin < 3
      color = [0.5 0.5 0.5]; % Grey
    end

    if nargin < 4
      linestyle = '--';
    end

    if nargin < 5
      drawObs = false;
    end

    % Number of individual obstacles ***TODO***
    nOb = 3;

    vOb = [4,4,4]; %TODO

    % Plot obstacle
    hSO = [];

    % if drawObs
    %     for i = 1 : nOb
    %         for j = 1 : vOb(i)
    %             hSO = [hSO line([lOb{i,j}(1),lOb{i,j+1}(1)] , [lOb{i,j}(2),lOb{i,j+1}(2)],'color',color,'LineStyle',linestyle)];
    %             hold on
    %         end
    %     end
    % end

    % Plot state trajectory
    hST = plot(state(:,2),state(:,4),'color',color,'LineStyle',linestyle);

    drawnow

end