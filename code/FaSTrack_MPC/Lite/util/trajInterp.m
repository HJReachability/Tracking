function refPath = trajInterp(path,delta_x)
% Interpolate a trajectory in x-y plane using the estimated speed
    refPath = [];
    for i = 1:length(path)-1
        pstart = path(i,:);
        pend = path(i+1,:);
        xdiff = abs(pend(1)-pstart(1));
        ydiff = abs(pend(2)-pstart(2));
        dis = distance(pstart,pend);
        if dis<delta_x
            refPath = [refPath; pstart; pend];           
            continue
        end
        if xdiff >= ydiff
            interval = delta_x*xdiff/dis;
            xref = pstart(1) : interval : pend(1);
            yref = interp1([pstart(1);pend(1)],[pstart(2);pend(2)],xref,'linear');
        else
            interval = delta_x*ydiff/dis;
            yref = pstart(2) : interval : pend(2);
            xref = interp1([pstart(2);pend(2)],[pstart(1);pend(1)],yref,'linear');            
        end
        refPath = [refPath; xref' yref'];
    end
end