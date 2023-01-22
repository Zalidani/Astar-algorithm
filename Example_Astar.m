clear all;

% A* algorithm 
% Example code
% By Zeid Al idani 25 may, 2020
%---------------------------------
% Start position
xStart = 18;
yStart = 1;

% Goal
xGoal = 6;
yGoal = 18;

% Initialize the maze
size_maze = 20;
maze = ones(size_maze);

% With the walls - Add walls, setting matrix positions to -1
maze(5,1:6) = -1;
maze(4:7,8) = -1;
maze(12,6:14) = -1;
maze(1:10,12) = -1;
maze(10:11,14) = -1;
maze(16,12:20) = -1;
maze(12:16,14) = -1;
maze(14:20,8) = -1;
maze(5:10,4) = -1;
maze(16,4:8) = -1;

% Apply the Astar algorithm to find the optimal path
OptimalPath = Astar(xStart,yStart,xGoal,yGoal,maze);

% Find all the walls in the maze
walls = find(maze == -1);

% Set them to -10 (for coloring purposes)
for j = 1:length(walls)
    maze(walls(j)) = -10;
end

% Draw the optimal path in the maze
for i = 1:length(OptimalPath)
    maze(OptimalPath(i,2),OptimalPath(i,1)) = 10;
end

% Change the color of the start position so it is visible in the maze 
maze(yStart,xStart) = 5;

% Display the maze with optimal path
imagesc(maze)

    
    