function [last_err] = KrigingTraverseMaze_multirobot(arr, A, B, C, D, E, numIter, numRobots)
    % Initialize arrays to store robot positions and errors
    robot_positions = cell(numRobots, 1);
    curr_robot_positions = zeros(numRobots, 2);
    robot_err = zeros(numRobots, numIter);

    % Define a set of unique colors for plotting
    colors = hsv(numRobots);

    [X,Y,Z] = createInitialSamplePts(arr);

    sizeArr = size(arr);
    [distances, rows, columns] = calculateDistances(sizeArr);

    startRow = round(mean([1, rows]));
    startCol = round(mean([1, columns]));

    err = zeros([1 numIter]);
    uncert = zeros([1 numIter]);

    x = [startRow; startRow; startRow + 1];
    y = [startCol; startCol + 1; startCol + 1];
    ind = sub2ind(sizeArr, x, y);
    z = arr(ind);

    path = [];
    count = 1;

    for robotIdx = 1:numRobots
        curr_robot_positions(robotIdx, :) = [startRow + 1 startCol + 1];
        robot_positions{robotIdx} = [robot_positions{robotIdx}; [startRow + 1 startCol + 1]];
    end

    % Initialize a figure for plotting
        figure;
        hold on;
    
    for i = 1:numIter
        for robotIdx = 1:numRobots
            if i == 1 || mod(i, 10) == 0
                vstruct = calculateVariogram(x, y, z);
                [Zhat, Zvar] = performKriging(vstruct, x, y, z, X, Y);
        
                maze = calculateCostMetric(Zhat, Zvar, distances, curr_robot_positions(robotIdx, :), curr_robot_positions, arr, A, B, C, D, E);
                [row_targ, col_targ] = determineTargetPosition(x, y, curr_robot_positions(robotIdx, :), maze);
                [path, ~, ~, ~] = Astar(maze, sizeArr, curr_robot_positions(robotIdx, :), [row_targ, col_targ]);
                count = 1;
            end
        
            sz = size(path);
            if ~isempty(path) && count < sz(1)
                curr_robot_positions(robotIdx, :) = path(end-count, :);
            end
            count = count + 1;
            x(end + 1) = curr_robot_positions(robotIdx, 1);
            y(end + 1) = curr_robot_positions(robotIdx, 2);
            z(end + 1) = arr(curr_robot_positions(robotIdx, 2), curr_robot_positions(robotIdx, 1));
            [curr_err, ~] = calculate_total_error_uncert(Zhat, Zvar, arr);
            robot_err(robotIdx, i) = curr_err;

            % Update robot position
            robot_positions{robotIdx} = [robot_positions{robotIdx}; curr_robot_positions(robotIdx, :)];
            plot(curr_robot_positions(robotIdx, 1), curr_robot_positions(robotIdx, 2), 'o', 'MarkerFaceColor', colors(robotIdx, :));
            drawnow;
        end
    end
    % Calculate the total error for all robots
    last_err = sum(robot_err(:).^2);

    % Add legend with robot colors
    legend(cellstr(num2str((1:numRobots)')), 'Location', 'Best');
    hold off;
end


function [X, Y, Z] = createInitialSamplePts(arr)
    sq_size = size(arr);
    n = sq_size(1);
    [X,Y] = meshgrid(0:n-1);
    Z = arr;
end


function [distances, rows, columns] = calculateDistances(sizeArr)
    rows = sizeArr(1);
    columns = sizeArr(2);
    midRow = mean([1, rows]);
    midCol = mean([1, columns]);
    distances = dist_from_pt(midRow, midCol, [rows columns]);
end

function vstruct = calculateVariogram(x, y, z)
    v = variogram([x y], z, 'plotit', false, 'maxdist', 100);
    [~,~,~,vstruct] = variogramfit(v.distance, v.val, [], [], [], 'model', 'stable', 'plotit', false);
end

function [Zhat, Zvar] = performKriging(vstruct, x, y, z, X, Y)
    [Zhat, Zvar] = kriging(vstruct, x, y, z, X, Y);
end

function closest_distances = findClosestDistances(grid_rows, grid_cols, points_of_interest)
    
    % Create grid coordinates using meshgrid
    [X, Y] = meshgrid(1:grid_cols, 1:grid_rows);
    
    closest_distances = zeros(grid_rows, grid_cols);
    
    for point = (points_of_interest')
        
        % Calculate distance from each cell to the point of interest
        distances = sqrt((X - point(1)).^2 + (Y - point(2)).^2);
        
        % Update closest_distances with the minimum distances
        closest_distances = min(closest_distances, distances);
    end
end


function maze = calculateCostMetric(Zhat, Zvar, distances, curr_position, curr_robot_positions, arr, A, B, C, D, E)
    [grid_rows, grid_cols] = size(arr);
    dist_from_cur = dist_from_pt(curr_position(1), curr_position(2), size(arr));
    closest_distances = findClosestDistances(grid_rows, grid_cols, curr_robot_positions);
    metric = (Zhat.^A) .* (Zvar.^B) .* ((1./distances).^C) .* ((1./dist_from_cur).^D) .* ((1./closest_distances).^E);
    cost = 1./metric;
    maze = cost;
end


function [dist_mat] = dist_from_pt(x_ind, y_ind, mat_sz)
    rows = mat_sz(1);
    columns = mat_sz(2);
    [X,Y] = meshgrid(1:columns, 1:rows);
    dist_mat = sqrt((X - x_ind) .^ 2 + (Y - y_ind) .^ 2);
end

function [row_targ, col_targ] = determineTargetPosition(x, y, curr_position, maze)
    [~, ind_targ] = max(maze,[],"all");
    [row_targ, col_targ] = ind2sub(size(maze), ind_targ);
    if x(end) == curr_position(1) && y(end) == curr_position(2)
        row_targ = randi([1, size(maze, 1)]);
        col_targ = randi([1, size(maze, 2)]);
    end
end

function [tot_err, tot_uncert] = calculate_total_error_uncert(Zhat, Zvar, arr)
    tot_err = sum((Zhat - arr).^2, "all");
    tot_uncert = sum(Zvar, "all");
end
