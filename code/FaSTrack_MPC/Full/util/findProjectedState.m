function index = findProjectedState(x0, refPath)

    count = 0;
    minDist = 1000;
    for i = 1:length(refPath)
        if count >= 5
            break
        end
        if distance(x0,refPath(i,:)) < minDist
            minDist = distance(x0,refPath(i,:));
            index = i;
        else
            count = count + 1;
        end
    end

end