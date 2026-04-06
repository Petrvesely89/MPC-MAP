function [target, path, finished] = get_target(estimated_pose, path)
%GET_TARGET Summary of this function goes here
finished = false;
switch_distance = 0.3;

if isempty(path)
    target = [];
    finished = true;
    return;
end

distance = norm(estimated_pose(1:2) - path(1,:));

if distance < switch_distance
    if size(path,1) > 1
        path(1,:) = [];
    else
        target = path(1,:);
        finished = true;
        return;
    end
end

target = path(1,:);

end

