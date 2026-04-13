function [new_pose] = predict_pose(old_pose, motion_vector, read_only_vars)
%PREDICT_POSE Summary of this function goes here

v = sum(motion_vector) / 2;
w = (motion_vector(1) - motion_vector(2)) / read_only_vars.agent_drive.interwheel_dist;
dt = read_only_vars.sampling_period;

x_new = old_pose(1) + v * cos(old_pose(3)) * dt;
y_new = old_pose(2) + v * sin(old_pose(3)) * dt;
theta_new = old_pose(3) + w * dt;

% Gaussian motion noise
sigma_x = 0.015;
sigma_y = 0.015;
sigma_theta = 0.04;

x_new = x_new + sigma_x * randn;
y_new = y_new + sigma_y * randn;
theta_new = theta_new + sigma_theta * randn;

theta_new = wrapToPi(theta_new);

new_pose = [x_new, y_new, theta_new];
end


