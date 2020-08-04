function virtQuadNext = findNextState(x0,refPath,dt,vss)
% find the next (virtual) state to track
    
    x0 = x0';

    k = findProjectedState(x0, refPath);
    xStart = refPath(k,1);
    yStart = refPath(k,2);
    
    dist_des = vss*dt;    
    dist = 0;
    flag = false;
    for i = k:length(refPath)-1
        dist = dist + distance(refPath(i,:),refPath(i+1,:));
        if dist > dist_des
            xNext = refPath(i+1,1);
            yNext = refPath(i+1,2);
            flag = true;
            break
        end
    end
    if ~flag
        xNext = refPath(end,1);
        yNext = refPath(end,2);
    end
    
    vxNext = (xNext - xStart)/dt;
    vyNext = (yNext - yStart)/dt;
    
    virtQuadNext = [xNext;vxNext;yNext;vyNext];
    
end