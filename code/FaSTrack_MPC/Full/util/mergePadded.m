function padded_obs = mergePadded(padded_obs,i,j)
% Merge two overlapped obstacles
    pxmax = -1;
    pxmin = 100;
    xmax = [];
    xmin = [];
    points = [padded_obs{i}; padded_obs{j}];
    
    for k = 1:length(points(:,1))      
        if points(k,1) < pxmin
            xmin = points(k,:);
            pxmin = points(k,1);
        elseif points(k,1) == pxmin
            xmin = [xmin; points(k,:)];
        end
        if points(k,1) > pxmax
            xmax = points(k,:);
            pxmax = points(k,1);
        elseif points(k,1) == pxmax
            xmax = [xmax; points(k,:)];
        end
    end
    
    pll = [xmin(1,1) min(xmin(:,2))];
    pul = [xmin(1,1) max(xmin(:,2))];
    plr = [xmax(1,1) min(xmax(:,2))];
    pur = [xmax(1,1) max(xmax(:,2))];
    
    padded_obs{j} = [];
    padded_obs{i} = [pll;plr;pur;pul];
    
end