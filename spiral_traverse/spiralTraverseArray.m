function spiralTraverseArray(arr)
    [rows, cols] = size(arr);
    centerRow = ceil(rows / 2);
    centerCol = ceil(cols / 2);
    
    % Define the four directions: right, down, left, up
    directions = [0 1; 1 0; 0 -1; -1 0];
    % Initialize variables
    currentRow = centerRow;
    currentCol = centerCol;
    currentDir = 1;  % Start moving to the right
    stepsToMove = 1;  % Number of steps to take in current direction
    stepsCounted = 0;  % Number of steps taken in current direction

    [X,Y,Z] = createInitialSamplePts(arr);
    subplot(1,2,1);
    plotTrueArena(X, Y, Z);

    x = [centerRow; centerRow; centerRow + 1];
    y = [centerCol; centerCol + 1; centerCol + 1];
    ind = sub2ind([rows cols], x, y);
    z = arr(ind);


    subplot(1,2,2)
    hold on
    title('Mapped Arena') 
    axis image; axis xy
    
    % Traverse the array
    for i = 1 : numel(arr)
        if i == 1 || mod(i, 10) == 0
            vstruct = calculateVariogram(x, y, z);
            [Zhat, ~] = performKriging(vstruct, x, y, z, X, Y);
            imagesc(X(1,:),Y(:,1),Zhat);
        end
        
        fprintf(currentRow + ", "+ currentCol+ "\n");  % Output the element
        stepsCounted = stepsCounted + 1;
        
        % Move to the next position
        currentRow = currentRow + directions(currentDir, 1);
        currentCol = currentCol + directions(currentDir, 2);
        
        % Check if direction needs to be changed
        if stepsCounted == stepsToMove
            % Change direction clockwise (right -> down -> left -> up)
            currentDir = mod(currentDir, 4) + 1;
            stepsCounted = 0;
            
            % Update steps to move for the next direction
            if currentDir == 1 || currentDir == 3
                stepsToMove = stepsToMove + 1;  % Same direction twice
            end
        end
        
        % Break the loop if out of bounds
        if currentRow < 1 || currentRow > rows || currentCol < 1 || currentCol > cols
            break;
        end

        x(end + 1) = currentRow;
        y(end + 1) = currentCol;
        z(end + 1) = arr(currentRow, currentCol);
        plot(x, y)
        F(i) = getframe(gcf);
        drawnow
        plotCurrentPosition(x, y);
    end
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


function plotCurrentPosition(x, y)
    plot(x, y)
end