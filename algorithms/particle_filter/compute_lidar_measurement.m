function [measurement] = compute_lidar_measurement(map, pose, lidar_config)
%COMPUTE_LIDAR_MEASUREMENT Simulate lidar measurement for a particle

measurement = zeros(1, length(lidar_config));

for i = 1:length(lidar_config)
    direction = wrapToPi(pose(3) + lidar_config(i));
    intersections = ray_cast(pose(1:2), map.walls, direction);

    if isempty(intersections)
        measurement(i) = inf;
    else
        dis = sqrt((intersections(:,1) - pose(1)).^2 + (intersections(:,2) - pose(2)).^2);
        measurement(i) = min(dis);
    end
end
end
