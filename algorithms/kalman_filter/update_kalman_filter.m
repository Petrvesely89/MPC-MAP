function [mu, sigma] = update_kalman_filter(read_only_vars, public_vars)
%UPDATE_KALMAN_FILTER Summary of this function goes here

mu = public_vars.mu;
sigma = public_vars.sigma;

% ===== Task 4: GNSS-based initialization =====
if read_only_vars.counter < 50
    return;
elseif read_only_vars.counter == 50
    mu = [mean(read_only_vars.gnss_history(:,1));
          mean(read_only_vars.gnss_history(:,2));
          mu(3)];

    sigma = [0.5 0   0;
             0   0.5 0;
             0   0   pi^2];
end


% I. Prediction
u = public_vars.motion_vector;
[mu, sigma] = ekf_predict(mu, sigma, u, public_vars.kf, read_only_vars.sampling_period);

% II. Measurement
z = read_only_vars.gnss_position(:);
[mu, sigma] = kf_measure(mu, sigma, z, public_vars.kf);

end

