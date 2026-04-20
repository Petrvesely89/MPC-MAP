function [public_vars] = init_kalman_filter(read_only_vars, public_vars)
%INIT_KALMAN_FILTER Summary of this function goes here

public_vars.kf.C = [1 0 0;
                    0 1 0];

public_vars.kf.R = [0.003 0    0;
                    0    0.003 0;
                    0    0    0.001];

public_vars.kf.Q = [0.2911  -0.0095;
                   -0.0095   0.2535];

public_vars.mu = [0;
                  0;
                  0];

public_vars.sigma = [1   0   0;
                     0   1   0;
                     0   0  pi^2];

public_vars.gnss_init_data = [];
public_vars.kf_initialized = false;

end

