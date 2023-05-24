function [path, open_array, cost_array, from_array] = Astar(maze, maze_size, start_position, goal_position)
    open_array = zeros(maze_size(1), maze_size(2));
    cost_array = inf(maze_size(1), maze_size(2));
    from_array = zeros(maze_size(1), maze_size(2), 2);
    
    open_array(start_position(1), start_position(2)) = 1;
    cost_array(start_position(1), start_position(2)) = 0;

    while true
        % Find the lowest-cost member within the open set
        lowest_cost = min(cost_array(open_array == 1));
        [lowest_member_rows, lowest_member_cols] = find(cost_array == lowest_cost & open_array == 1, 1);
        current_node = [lowest_member_rows(1), lowest_member_cols(1)];

        if isequal(current_node, goal_position)
            break;
        end

        open_array(current_node(1), current_node(2)) = -1;

        % Add neighbors of current_node to the open set
        for dx = -1:1
            for dy = -1:1
                if dx == 0 && dy == 0
                    continue;
                end
                
                xx = dx + current_node(1);
                yy = dy + current_node(2);

                if xx >= 1 && xx <= maze_size(1) && yy >= 1 && yy <= maze_size(2) && open_array(xx, yy) == 0
                    open_array(xx, yy) = 1;
                    h_cost = heuristic([xx, yy], goal_position);
                    path_cost = cost_array(current_node(1), current_node(2)) + maze(xx, yy) + heuristic([xx, yy], goal_position);

                    
                    if path_cost < cost_array(xx, yy)
                        cost_array(xx, yy) = path_cost;
                        from_array(xx, yy, :) = current_node;
                    end
                end
            end    
        end
    end

    % Generate the path from goal to start by following the from_array
    path = [goal_position];
    while ~isequal(path(end, :), start_position)
        path = [path; reshape(from_array(path(end, 1), path(end, 2), :), [1, 2])];
    end
end
