function [path] = astar(read_only_vars, public_vars)
%ASTAR Summary of this function goes here

    map = read_only_vars.discrete_map.map;

    % Task 2: clearance 0.2 m
    clearance = 0.2;
    
    resolution_x = read_only_vars.map.limits(3) / size(map, 2);
    resolution_y = read_only_vars.map.limits(4) / size(map, 1);
    resolution = min(resolution_x, resolution_y);
    
    radius_cells = ceil(clearance / resolution);
    
    map = conv2(double(map ~= 0), ones(2*radius_cells + 1), 'same') > 0;
    
    [rows, cols] = size(map);

    goal_pos = [
    read_only_vars.discrete_map.goal(2), read_only_vars.discrete_map.goal(1)];

    start_xy = public_vars.estimated_pose(1:2);
    %start_xy = [1.2,1];
    start_pos = world_to_grid(start_xy, read_only_vars);
    start_pos = find_nearest_free_cell(map, start_pos);
    goal_pos = find_nearest_free_cell(map, goal_pos);

    moves = [0  1; 1  0; 0 -1; -1  0];

    open_list = [];
    closed_list = false(rows, cols);

    g_cost = inf(rows, cols);
    f_cost = inf(rows, cols);
    parent = zeros(rows, cols, 2);

    g_cost(start_pos(1), start_pos(2)) = 0;
    f_cost(start_pos(1), start_pos(2)) = heuristic(start_pos, goal_pos);

    open_list = [start_pos, f_cost(start_pos(1), start_pos(2))];

    % figure;
    % imagesc(map);
    % set(gca, 'YDir', 'normal');
    % axis equal;
    % axis tight;
    % colormap(gray);
    % hold on;
    % 
    % % vykresli start a goal
    % plot(start_pos(2), start_pos(1), 'go', 'MarkerSize', 10, 'LineWidth', 2);
    % plot(goal_pos(2), goal_pos(1), 'rx', 'MarkerSize', 10, 'LineWidth', 2);
    % 
    % title('Grid map used by A*');

    while ~isempty(open_list)

        [~, idx] = min(open_list(:, 3));
        current = open_list(idx, 1:2);
        open_list(idx, :) = [];

        if closed_list(current(1), current(2))
            continue;
        end

        closed_list(current(1), current(2)) = true;


        if isequal(current, goal_pos)
            path_grid = reconstruct_path(parent, start_pos, goal_pos);
            path = grid_to_world_path(path_grid, read_only_vars);
            return;
        end


        for i = 1:size(moves, 1)

            neighbor = current + moves(i, :);

            if neighbor(1) < 1 || neighbor(1) > rows || ...
               neighbor(2) < 1 || neighbor(2) > cols
                continue;
            end


            if map(neighbor(1), neighbor(2)) ~= 0 || ...
               closed_list(neighbor(1), neighbor(2))
                continue;
            end

            tentative_g = g_cost(current(1), current(2)) + 1;


            if tentative_g < g_cost(neighbor(1), neighbor(2))

                parent(neighbor(1), neighbor(2), :) = current;

                g_cost(neighbor(1), neighbor(2)) = tentative_g;
                f_cost(neighbor(1), neighbor(2)) = ...
                    tentative_g + heuristic(neighbor, goal_pos);


                open_list = [
                    open_list;
                    neighbor, f_cost(neighbor(1), neighbor(2))
                ];
            end
        end
    end

    disp("No path found");
    path = [];

end


function h = heuristic(pos, goal)
    h = norm(pos - goal);
end


function grid_pos = world_to_grid(pos_xy, read_only_vars)

    limits = read_only_vars.map.limits;
    map = read_only_vars.discrete_map.map;
    [rows, cols] = size(map);

    x = pos_xy(1);
    y = pos_xy(2);

    row = round(rows * y / limits(4));
    col = round(cols * x / limits(3));

    row = max(1, min(rows, row));
    col = max(1, min(cols, col));

    grid_pos = [row, col];

end


function path = reconstruct_path(parent, start_pos, goal_pos)

    path = goal_pos;

    while ~isequal(path(1, :), start_pos)

        current = path(1, :);

        prev = parent(current(1), current(2), :);
        prev = prev(:)';

        if all(prev == 0)
            path = [];
            return;
        end

        path = [prev; path];
    end

end

function path_world = grid_to_world_path(path_grid, read_only_vars)

    limits = read_only_vars.map.limits;
    map = read_only_vars.discrete_map.map;
    [rows, cols] = size(map);

    path_world = zeros(size(path_grid, 1), 2);

    for i = 1:size(path_grid, 1)
        row = path_grid(i, 1);
        col = path_grid(i, 2);

        x = col * limits(3) / cols;
        y = row * limits(4) / rows;

        path_world(i, :) = [x, y];
    end

end

function free_pos = find_nearest_free_cell(map, pos)

    [rows, cols] = size(map);

    if map(pos(1), pos(2)) == 0
        free_pos = pos;
        return;
    end

    for radius = 1:30
        for dr = -radius:radius
            for dc = -radius:radius

                r = pos(1) + dr;
                c = pos(2) + dc;

                if r >= 1 && r <= rows && c >= 1 && c <= cols
                    if map(r,c) == 0
                        free_pos = [r, c];
                        return;
                    end
                end

            end
        end
    end

    free_pos = pos;

end
