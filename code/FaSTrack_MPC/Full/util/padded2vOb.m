function vOb = padded2vOb(padded_obs)
% get obstacle vOb representation from padded_obs

    nOb = length(padded_obs);
    Inf = 10000;
    vOb = cell(nOb,4);
    for i = 1:nOb
        if isempty(padded_obs{i})
            vOb{i,1} = [Inf,Inf];
            vOb{i,2} = [Inf+1,Inf];
            vOb{i,3} = [Inf+1,Inf+1];
            vOb{i,4} = [Inf,Inf+1];
        else
            for j = 1:4
                vOb{i,j} = padded_obs{i}(j,:);
            end      
        end            
    end
end