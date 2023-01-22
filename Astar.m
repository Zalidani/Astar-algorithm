function OptimalPath = Astar(xStart,yStart,xGoal,yGoal,maze)

    % A* algorithm
    % Version 1.0
    % By Zeid Al idani 25 may, 2020
    %-------------------------------
    
    size_maze = size(maze,1); % Size of the maze
    PathTrav=0;               % Path travelled, initially zero
    prevPathTrav = 0;         % Keep track of the previous travelled path
    GoalDistance=norm([xGoal,yGoal] - [xStart,yStart]); % Hueristic distance from start to finish

    % Set the starting position to current node
    Current_xNode = xStart;   
    Current_yNode = yStart; 
    
    % Initialize the set with the open nodes and keep track of travelled
    % distance, distance to goal and the f value (goal distance + path
    % travelled)
    OpenSet(1,:)=[Current_xNode,Current_yNode,xStart,yStart,PathTrav,GoalDistance,GoalDistance+PathTrav]; 
    % Initialize the closed set
    ClosedSet = [-1,-1,-1,-1,-1,-1,-1];
    %----------------------------------------------------------------------
    % Begin the algorithm 
    while((Current_xNode ~= xGoal || Current_yNode ~= yGoal))
        
        % Find the adjacent nodes
        neigh(1:8,1:2) = [Current_yNode+[-1;0;1;-1;1;-1;0;1] Current_xNode+[-1;-1;-1;0;0;1;1;1] ];          % Find all adjacent nodes
        neigh = neigh(all(neigh,2) & neigh(:,1) <= size_maze & neigh(:,2) <= size_maze,:);
        AdjNodes = (neigh(:,2)-1)*size_maze + neigh(:,1);                                   % And store them in AdjNodes
        [Adj_yNode,Adj_xNode] = ind2sub(size(maze),AdjNodes);  
        
        % Check if any of them are walls or in the closed set and 'remove' the ones that are
        for k = 1:length(AdjNodes)
            if (ismember([Adj_xNode(k),Adj_yNode(k)],ClosedSet(:,1:2),'rows') == 1 || maze(Adj_yNode(k),Adj_xNode(k)) == -1)
                AdjNodes(k) = 0;
            end
        end
        
        % Update their values for goal distance, travelled distance and f
        % value
        for i = 1:length(AdjNodes)
                if(AdjNodes(i) ~= 0)  
                [Adj_yNode,Adj_xNode] = ind2sub(size(maze),AdjNodes(i));  
                PathTrav=prevPathTrav+norm([Current_xNode,Current_yNode] - [Adj_xNode,Adj_yNode]);
                GoalDistance=norm([xGoal,yGoal] - [Adj_xNode,Adj_yNode]);
                    if (ismember([Adj_xNode,Adj_yNode],OpenSet(:,1:2),'rows') == 0)
                    OpenSet(end+1,:)=[Adj_xNode,Adj_yNode,Current_xNode,Current_yNode,PathTrav,GoalDistance,GoalDistance+PathTrav];
                    elseif (OpenSet(find(ismember(OpenSet(:,1:2),[Adj_xNode,Adj_yNode],'rows')),7) > GoalDistance)
                        OpenSet(find(ismember(OpenSet(:,1:2),[Adj_xNode,Adj_yNode],'rows')),7) = GoalDistance+PathTrav;
                        OpenSet(find(ismember(OpenSet(:,1:2),[Adj_xNode,Adj_yNode],'rows')),5) = PathTrav;
                    end
                end
        end
          
          % Find the row with the current position in the open set and move
          % it to the closed set and remove it from open set
          row_xNode=find(ismember(OpenSet(:,1:2),[Current_xNode,Current_yNode],'rows'));  
          ClosedSet(end+1,:) = OpenSet(row_xNode,:);
          OpenSet(row_xNode,:) = []; 
          
          % Find the node with the lowest f value and it will be the new
          % current node
          [minf,row_minf] = min(OpenSet(:,7));           
          Current_xNode = OpenSet(row_minf,1);
          Current_yNode = OpenSet(row_minf,2);
          % Keep track of the path travelled from start
          prevPathTrav = OpenSet(row_minf,5);
          
    end

    % Move all the nodes that lead up to the goal node to the optimal path
    OptimalPath = [xGoal,yGoal];
    OptimalPath(end+1,:) = ClosedSet(end,1:2);
    OptimalPath(end+1,:) = ClosedSet(end,3:4);

    while (OptimalPath(end,1) ~= xStart || OptimalPath(end,2) ~= yStart)
        row_Prev_xNode=find(ismember(ClosedSet(:,1:2),OptimalPath(end,:),'rows'));
        OptimalPath(end+1,:) = ClosedSet(row_Prev_xNode,3:4);       
    end

end

    
    
    