clc;
clear;
close all;

load("perlin.mat")
% create random field with autocorrelation
sq_size = size(I);
n = sq_size(1);
[X,Y] = meshgrid(0:n-1);
Z = I;

% sample the field
x = rand(n,1)*n-1;
y = rand(n,1)*n-1;
z = interp2(X,Y,Z,x,y, 'makima');

% plot the random field
subplot(2,2,1)
imagesc(X(1,:),Y(:,1),Z); axis image; axis xy
hold on
plot(x,y,'.k')
title('random field with sampling locations')

% calculate the sample variogram
v = variogram([x y],z,'plotit',false,'maxdist',100);
% and fit a spherical variogram
subplot(2,2,2)
[~,~,~,vstruct] = variogramfit(v.distance,v.val,[],[],[],'model','stable');
title('variogram')

% now use the sampled locations in a kriging
[Zhat,Zvar] = kriging(vstruct,x,y,z,X,Y);
subplot(2,2,3)
imagesc(X(1,:),Y(:,1),Zhat); axis image; axis xy
title('kriging predictions')
subplot(2,2,4)
contour(X,Y,Zvar); axis image
title('kriging variance')


% Distance to Center
rows = 181;
columns = 181;
distances = zeros(rows, columns);
midRow = mean([1, rows])
midCol = mean([1, columns])
for col = 1 : columns
	for row = 1 : rows
		distances(row, col) = sqrt((row - midRow) .^ 2 + (col - midCol) .^ 2);
	end
end

A = 1;
B = 1;
C = 1;

metric = A*Zhat + B*Zvar + C*(1./distances);
cost = 1./metric;

maze = cost;
maze_size = size(maze);
start_position = [20 5];
goal_position = [130 130];

figure(2)

[path, open_array, cost_array, from_array] = Astar(maze, maze_size, ...
    start_position, goal_position);

drawMaze(maze, maze_size, start_position, goal_position, path, ...
    open_array, cost_array, from_array);