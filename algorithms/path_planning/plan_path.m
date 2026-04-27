function [path] = plan_path(read_only_vars, public_vars)
%PLAN_PATH Summary of this function goes here

planning_required = 1;

if planning_required

    path = astar(read_only_vars, public_vars);

    path = smooth_path(path);

else

    %path = create_my_path();
    path = create_my_path_outdoor_1();

end
end

function path = create_my_path_outdoor_1()

% LINE
[x1, y1] = generate_line([2, 2], [2, 5], 15);

% ARC
[x2, y2] = generate_arc([2, 5], [5, 8], -1, 2.5, 25);

% LINE
[x3, y3] = generate_line([5, 8], [12, 8], 20);

% ARC
[x4, y4] = generate_arc([12, 8], [16, 5], -1, 4, 25);

% LINE
[x5, y5] = generate_line([16, 5], [16, 2], 12);



x = [x1; x2; x3; x4; x5];
y = [y1; y2; y3; y4; y5];

path = [x, y];

end


function path = create_my_path()

% SINE
[x1, y1] = generate_sine([1, 7.5], [7, 7.5], 0.3, 50);

% ARC
[x2, y2] = generate_arc([7, 7.5], [7, 5], -1, 1.25, 30);

% LINE
[x3, y3] = generate_line([7, 5], [3, 5], 40);

% ARC
[x4, y4] = generate_arc([3, 5], [3, 1], 1, 2, 30);

% LINE
[x5, y5] = generate_line([3, 1], [9, 1], 60);



x = [x1; x2; x3; x4; x5];
y = [y1; y2; y3; y4; y5];

path = [x, y];

end

function [x, y] = generate_line(p_start, p_end, n)
x = linspace(p_start(1), p_end(1), n)';
y = linspace(p_start(2), p_end(2), n)';
end

function [x, y] = generate_arc(p_start, p_end, direction, radius, n)
% direction = 1 (CCW) nebo -1 (CW)

dx = p_end(1) - p_start(1);
dy = p_end(2) - p_start(2);

mid = (p_start + p_end) / 2;
d = sqrt(dx^2 + dy^2);

if radius < d/2
    error('Radius too small');
end

h = sqrt(radius^2 - (d/2)^2);

perp = [-dy, dx] / norm([dx, dy]);

center = mid + direction * h * perp;

theta1 = atan2(p_start(2) - center(2), p_start(1) - center(1));
theta2 = atan2(p_end(2) - center(2), p_end(1) - center(1));

if direction == 1 && theta2 < theta1
    theta2 = theta2 + 2*pi;
elseif direction == -1 && theta2 > theta1
    theta2 = theta2 - 2*pi;
end

theta = linspace(theta1, theta2, n);

x = center(1) + radius*cos(theta);
y = center(2) + radius*sin(theta);

x = x(:);
y = y(:);

end

function [x, y] = generate_sine(p_start, p_end, amplitude, n)
x = linspace(p_start(1), p_end(1), n)';
length_x = p_end(1) - p_start(1);
y = p_start(2) + amplitude * sin(2*pi*(x - p_start(1)) / length_x);
end


