function [new_mu, new_sigma] = ekf_predict(mu, sigma, u, kf, sampling_period)
%EKF_PREDICT Summary of this function goes here

x = mu(1);
y = mu(2);
theta = mu(3);

vL = u(1);
vR = u(2);

L = 0.2; % value from read_only_vars.agent_drive.interwheel_dist
dt = sampling_period;

v = (vL + vR) / 2;
w = (vL - vR) / L;

x_new = x + v * cos(theta) * dt;
y_new = y + v * sin(theta) * dt;
theta_new = mod(theta + w * dt + pi, 2*pi) - pi;

new_mu = [x_new;
          y_new;
          theta_new];

G = [1 0 -v * dt * sin(theta);
     0 1  v * dt * cos(theta);
     0 0  1];


new_sigma = G * sigma * G' + kf.R;

end

