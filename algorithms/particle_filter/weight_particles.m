function [weights] = weight_particles(particle_measurements, lidar_distances)
%WEIGHT_PARTICLES Compute particle weights based on lidar error
lidar_distances = lidar_distances(:)';

error = sqrt(sum((particle_measurements - lidar_distances).^2, 2));

weights = 1 ./ (error + 1e-6); % avoid division by zero

weights = weights / sum(weights);

end
