function padded_obs = updatePadded(local_obs, track_err_vec, iter)
% Find four vertices of each padded obstacle, order(CCW):
% {[lower_left lower_right upper_right upper_left],...,...}

    padded_obs = cell(1,3);
    
    if iter <= 50
        track_err = track_err_vec(1);
    elseif 50<iter && iter<=80
        track_err = track_err_vec(2);
    else
        track_err = track_err_vec(3);
    end
    
    for i = 1:length(local_obs)
        if length(local_obs{i}) >= 3
            [pll,plr,pur,pul] = findVertices(local_obs{i});
            pll = pll + [-track_err,-track_err];
            plr = plr + [track_err,-track_err];
            pur = pur + [track_err,track_err];
            pul = pul + [-track_err,track_err];
            padded_obs{i} = [pll;plr;pur;pul];
        else
            padded_obs{i} = [];
        end
    end
end