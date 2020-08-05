function nextState = findNextStateOld(x0,refPath,delta_x)
% find next state to track
    dis = 0;
    
    for i = 1:length(refPath)
       if distance([x0(1) x0(2)],refPath(i,:)) < delta_x/10
           index = i;
           break
       end
    end
        
    for i = index:length(refPath)-1
        dis = dis + distance(refPath(i,:),refPath(i+1,:));
        if dis > delta_x
            nextState = refPath(i+1,:);
            return
        end
    end
    
    nextState = refPath(end,:);
    
end