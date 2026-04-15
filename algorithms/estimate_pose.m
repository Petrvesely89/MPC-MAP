function [estimated_pose] = estimate_pose(public_vars)
%ESTIMATE_POSE Estimate pose from particles

particles = public_vars.particles;
x = median(particles(:,1));
y = median(particles(:,2));
theta = atan2(mean(sin(particles(:,3))), mean(cos(particles(:,3))));
estimated_pose = [x, y, theta];
end
