function obs = refineVertex(obs)
% Avoid triangle shape for a single obstacle
% obs = [pll;plr;pur;pul];

    delta = 0.1;
    pll = obs(1,:);
    plr = obs(2,:);
    pur = obs(3,:);
    pul = obs(4,:);
    if pll(1) == plr(1)
        plr(1) = plr(1) + delta;
    end
    if plr(2) == pur(2)
        pur(2) = pur(2) + delta;
    end
    if pur(1) == pul(1)
        pul(1) = pul(1) - delta;
    end
    if pul(2) == pll(2)
        pul(2) = pul(2) + delta;
    end

    obs = [pll;plr;pur;pul];

end