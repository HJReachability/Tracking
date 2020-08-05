function lOb = getlOb(obs1,obs2,obs3)
% Convert obstacle vertices representation to cell representation

    lOb = {};
    for i = 1:length(obs1)
        lOb{1,i} = obs1(i,:)';
    end
    lOb{1,length(obs1)+1} = obs1(1,:)';
    for i = 1:length(obs2)
        lOb{2,i} = obs2(i,:)';
    end
    lOb{2,length(obs1)+1} = obs2(1,:)';
    for i = 1:length(obs3)
        lOb{3,i} = obs3(i,:)';
    end
    lOb{3,length(obs1)+1} = obs3(1,:)';

end