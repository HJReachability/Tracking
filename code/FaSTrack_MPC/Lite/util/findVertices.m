function [pll,plr,pur,pul] = findVertices(obs)
% For one individual obstacle, find its four vertices, order(CCW):
% [lower_left lower_right upper_right upper_left]
    xl = 0;
    xu = 0;
    pxmax = -1;
    pxmin = 100;
    xmax = [];
    xmin = [];
    points = [];
    for vertex = obs
        points = [points; vertex.point];
    end
    
    for i = 1:length(points(:,1))      
        if points(i,1) < pxmin
            xmin = points(i,:);
            pxmin = points(i,1);
        elseif points(i,1) == pxmin
            xmin = [xmin; points(i,:)];
        end
        if points(i,1) > pxmax
            xmax = points(i,:);
            pxmax = points(i,1);
        elseif points(i,1) == pxmax
            xmax = [xmax; points(i,:)];
        end
    end
    
    pll = [xmin(1,1) min(xmin(:,2))];
    pul = [xmin(1,1) max(xmin(:,2))];
    plr = [xmax(1,1) min(xmax(:,2))];
    pur = [xmax(1,1) max(xmax(:,2))];
end