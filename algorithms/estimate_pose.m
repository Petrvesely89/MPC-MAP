function [estimated_pose] = estimate_pose(public_vars)
%ESTIMATE_POSE Estimate pose from particles
if ~isempty(public_vars.particles)
    particles = public_vars.particles;
    x = median(particles(:,1));
    y = median(particles(:,2));
    theta = atan2(mean(sin(particles(:,3))), mean(cos(particles(:,3))));
    estimated_pose = [x, y, theta];
elseif ~isempty(public_vars.mu)
    estimated_pose = public_vars.mu';
else
    estimated_pose = nan(1,3);
end
end

