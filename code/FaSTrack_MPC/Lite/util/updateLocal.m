function local_obs = updateLocal(x0, global_obs, local_obs, sense_radius)
% Update local obstacles
    for i = 1:length(global_obs)
        for vertex = global_obs{i}
            if not(vertex.sensed) && norm(vertex.point-x0')<=sense_radius
                vertex.sensed = true;
                local_obs{i} = [local_obs{i} vertex];
            end
        end
    end
end