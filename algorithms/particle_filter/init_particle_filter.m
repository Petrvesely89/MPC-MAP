function [public_vars] = init_particle_filter(read_only_vars, public_vars)
%INIT_PARTICLE_FILTER Initialize particles randomly within GNSS-denied area

n = 1000; % number of particles

% Extract GNSS-denied area boundaries
poly = read_only_vars.map.gnss_denied;
xs = poly(1:2:end);
ys = poly(2:2:end);

xmin = min(xs);
xmax = max(xs);
ymin = min(ys);
ymax = max(ys);

% Generate random particle positions within the area
x = xmin + (xmax - xmin) * rand(n,1);
y = ymin + (ymax - ymin) * rand(n,1);

% Generate random orientations (angles between 0 and 2*pi)
theta = -pi + 2*pi * rand(n,1);

% Combine into particle matrix [x, y, theta]
public_vars.particles = [x, y, theta];

end

