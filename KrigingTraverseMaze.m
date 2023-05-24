function KrigingTraverseMaze(arr, A, B, C, D, numIter)
    [X,Y,Z] = createInitialSamplePts(arr);

    subplot(1,2,1);
    plotTrueArena(X, Y, Z);

    sizeArr = size(arr);
    [distances, rows, columns] = calculateDistances(sizeArr);

    startRow = mean([1, rows]);
    startCol = mean([1, columns]);

    err = zeros([1 numIter]);
    uncert = zeros([1 numIter]);

    x = [startRow; startRow; startRow + 1];
    y = [startCol; startCol + 1; startCol + 1];
    ind = sub2ind(sizeArr, x, y);
    z = arr(ind);


    subplot(1,2,2)
    hold on
    title('Mapped Arena') 
    axis image; axis xy

    path = [];
    count = 1;
    curr_position = [startRow + 1 startCol + 1];
    
    for i = 1:numIter
        
        if i == 1 || mod(i, 10) == 0
            vstruct = calculateVariogram(x, y, z);
            [Zhat, Zvar] = performKriging(vstruct, x, y, z, X, Y);
            imagesc(X(1,:),Y(:,1),Zhat, [min(arr, [], 'all') max(arr,[], 'all')]);

            maze = calculateCostMetric(Zhat, Zvar, distances, curr_position, arr, A, B, C, D);
            [row_targ, col_targ] = determineTargetPosition(x, y, curr_position, maze);
            [path, ~, ~, ~] = Astar(maze, sizeArr, curr_position, [row_targ, col_targ]);
            count = 1;
        end

        sz = size(path);
        if ~isempty(path) && count < sz(1)
            curr_position = path(end-count, :);
        end
        count = count + 1;
        x(end + 1) = curr_position(1);
        y(end + 1) = curr_position(2);
        z(end + 1) = arr(curr_position(1), curr_position(2));
        [curr_err, curr_uncert] = calculate_total_error_uncert(Zhat, Zvar, arr);
        err(i) = curr_err;
        uncert(i) = curr_uncert;
        plot(x, y)
        F(i) = getframe(gcf);
        drawnow
        plotCurrentPosition(x, y);

        F(i) = getframe(gcf);
        drawnow;
    end

    figure()
    yyaxis left
    plot(err)
    title('Metrics')
    xlabel('Step')
    ylabel('$\sum error^{2}$', 'interpreter', 'latex')
    yyaxis right
    plot(uncert)
    ylabel('$\sum$ Uncertainty', 'interpreter', 'latex')

    figure()
    imagesc(arr)
    hold on
    plot(x,y)
    axis image; axis xy
    write_to_video(F);
end

function [X, Y, Z] = createInitialSamplePts(arr)
    sq_size = size(arr);
    n = sq_size(1);
    [X,Y] = meshgrid(0:n-1);
    Z = arr;
end

function plotTrueArena(X, Y, Z)
    imagesc(X(1,:),Y(:,1),Z);
    axis image; axis xy;
    hold on;
    title('True Arena');
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

function maze = calculateCostMetric(Zhat, Zvar, distances, curr_position, arr, A, B, C, D)
    dist_from_cur = dist_from_pt(curr_position(1), curr_position(2), size(arr));
    metric = (Zhat.^A) .* (Zvar.^B) .* ((1./distances).^C) .* ((1./dist_from_cur).^D);
    cost = 1./metric;
    maze = cost;
end


function plotCurrentPosition(x, y)
    plot(x, y)
end

function write_to_video(F)
    writerObj = VideoWriter('myVideo', 'MPEG-4');
    writerObj.FrameRate = 30;
    % open the video writer
    open(writerObj);
    % write the frames to the video
    for i=1:length(F)
        % convert the image to a frame
        frame = F(i) ;    
        writeVideo(writerObj, frame);
    end
    % close the writer object
    close(writerObj);
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