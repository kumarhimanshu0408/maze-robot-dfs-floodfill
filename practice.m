clear; close all; clc;

%% PARAMETERS (Test-2)
N = 6;                % Maze grid size (N x N)
cellSize = 1.0;       % Physical size of one cell (meters)
dt = 0.05;            % Simulation time step (seconds)
maxSteps = 20000;     % Safety limit for simulation steps
poseTol = 0.03;       % Tolerance for reaching a cell's center (meters)
plotInterval = 0.02;  % Time between plot updates

%% MAZE DEFINITION
% This is the "Ground Truth" map, not visible to the robot.
% maze(r,c,:) = [N E S W], 1 = wall, 0 = open
maze = zeros(N,N,4);

% --- Outer walls ---
maze(:,1,4)=1; maze(:,N,2)=1; % West, East
maze(1,:,1)=1; maze(N,:,3)=1; % North, South

% --- Internal walls ---
% Each pair of lines defines one wall.
maze(1,2,3)=1; maze(2,2,1)=1;
maze(1,3,3)=1; maze(2,3,1)=1;
maze(2,1,3)=1; maze(3,1,1)=1;
maze(2,2,3)=1; maze(3,2,1)=1;
maze(2,4,3)=1; maze(3,4,1)=1;
maze(2,5,3)=1; maze(3,5,1)=1;
maze(3,4,3)=1; maze(4,4,1)=1;
maze(4,2,3)=1; maze(5,2,1)=1;
maze(4,5,3)=1; maze(5,5,1)=1;
maze(5,2,3)=1; maze(6,2,1)=1;
maze(5,3,3)=1; maze(6,3,1)=1;
maze(5,4,3)=1; maze(6,4,1)=1;
maze(5,5,3)=1; maze(6,5,1)=1;
maze(5,6,3)=1; maze(6,6,1)=1;
maze(1,2,2)=1; maze(1,3,4)=1;
maze(1,4,2)=1; maze(1,5,4)=1;
maze(2,3,2)=1; maze(2,4,4)=1;
maze(2,5,2)=1; maze(2,6,4)=1;
maze(3,2,2)=1; maze(3,3,4)=1;
maze(3,5,2)=1; maze(3,6,4)=1;
maze(4,1,2)=1; maze(4,2,4)=1;
maze(4,3,2)=1; maze(4,4,4)=1;
maze(4,5,2)=1; maze(4,6,4)=1;
maze(5,1,2)=1; maze(5,2,4)=1;
maze(5,3,2)=1; maze(5,4,4)=1;
% ------------------------------------------------------------
startCell = [2,6];   
goalCell  = [2,1];   

%% INITIAL STATE
% Calculate continuous (x,y) position from discrete (row,col)
x = (startCell(2)-1) + 0.5;
y = (N - startCell(1)) + 0.5;
theta = 0; % Initial angle (0 = facing East)
pose = [x; y; theta]; % Robot's continuous state vector [x; y; theta]

% robot_map is the robot's "Memory", initialized to -1 (Unknown)
% 1 = Wall, 0 = No Wall, -1 = Unknown
robot_map = -1 * ones(N, N, 4); 

visitedCells = false(N,N); % Tracks visited cells for DFS
exploreStack = startCell;  % Stack for DFS exploration
traj = pose'; % Stores the history of the robot's pose

% Sense walls at the start and mark as visited
robot_map(startCell(1), startCell(2), :) = sense_walls(maze, startCell);
visitedCells(startCell(1), startCell(2)) = true;
goalFound = false; % Flag for exploration

%% PLOTTING SETUP
fig = figure('Name','Maze Exploration & Flood-Fill Path','NumberTitle','off','Color',[1 1 1]);
ax = axes(fig); hold(ax,'on'); axis(ax,'equal'); axis(ax,[0 N 0 N]);
xlabel('x (m)'); ylabel('y (m)');
title('Exploration Phase');
draw_true_maze(ax, maze, N); % Draw "ground truth" maze in grey
text(startCell(2)-0.5, N-startCell(1)+0.5, 'S','FontWeight','bold','HorizontalAlignment','center');
text(goalCell(2)-0.5, N-goalCell(1)+0.5, 'G','FontWeight','bold','HorizontalAlignment','center');

% Create plot handles for visited cells (for fast updating)
hVisited = gobjects(N,N);
for r=1:N
    for c=1:N
        hVisited(r,c) = rectangle('Position',[c-1, N-r, 1, 1],'FaceColor','none','EdgeColor','none');
    end
end
% Create plot handle for the robot's triangular patch
hRobot = patch(0,0,'r','FaceColor',[0.85,0.2,0.2],'EdgeColor','k');
drawnow;

%% CONTROLLER PARAMETERS
K_rho = 1.6;    % Proportional gain for speed (v)
K_alpha = 6.0;  % Proportional gain for turning (omega)
max_v = 0.6;    % Max linear speed (m/s)
max_omega = 4.0;% Max angular speed (rad/s)

%% ---------------- EXPLORATION (DFS) ----------------
% PHASE 1: Robot explores the maze using DFS to build its 'robot_map'.
step = 0;
lastPlotTime = tic;
currentCell = startCell;

while step < maxSteps
    step = step + 1;
    
    % --- 1. Continuous Robot Control (Move to cell center) ---
    center = cell_center_pose(currentCell, N);
    distToCenter = hypot(pose(1)-center(1), pose(2)-center(2));
    
    if distToCenter > poseTol
        % --- Not at center: Keep moving ---
        
        % Proportional controller
        dx = center(1) - pose(1);
        dy = center(2) - pose(2);
        target_theta = atan2(dy, dx);             % Target angle
        e_theta = angdiff(target_theta, pose(3)); % Angle error
        rho = hypot(dx, dy);                      % Distance error
        
        % Calculate control commands
        v = K_rho * rho * (abs(e_theta) < pi/2); % Only move forward
        omega = K_alpha * e_theta;
        
        % Saturate controls
        v = max(min(v, max_v), -max_v);
        omega = max(min(omega, max_omega), -max_omega);
        
        % Update pose using differential drive model
        pose(1) = pose(1) + v*cos(pose(3))*dt;
        pose(2) = pose(2) + v*sin(pose(3))*dt;
        pose(3) = wrapToPi(pose(3) + omega*dt);
        traj(end+1,:) = pose';
        
    else
        % --- 2. Discrete Maze Logic (DFS) ---
        % Arrived at cell center. Stop, sense, and decide next move.
        v = 0; omega = 0;
        
        % Sense walls and update "memory" (robot_map)
        robot_map(currentCell(1), currentCell(2), :) = sense_walls(maze, currentCell);
        visitedCells(currentCell(1), currentCell(2)) = true;
        
        % Update plot
        if toc(lastPlotTime) > plotInterval
            update_plot(ax, robot_map, visitedCells, N, hVisited, @update_discovered_lines);
            lastPlotTime = tic;
        end
        
        % Check if this is the goal
        if isequal(currentCell, goalCell)
            goalFound = true;
        end
        
        % Core DFS Logic:
        % Find an accessible, unvisited neighbor
        nbrs = accessible_unvisited_neighbors(robot_map, currentCell, visitedCells, N);
        
        if ~isempty(nbrs)
            % Neighbor found: Push to stack and move to it (go deeper)
            exploreStack = [exploreStack; nbrs(1,:)];
            currentCell = nbrs(1,:);
        else
            % Dead end: Pop from stack (backtrack)
            exploreStack(end,:) = [];
            if isempty(exploreStack)
                break; % Stack is empty, exploration is complete
            end
            currentCell = exploreStack(end,:); % Go to previous cell
        end
    end
    
    % Update robot visualization
    if mod(step,3) == 0
        update_robot_patch(hRobot, pose, 0.20);
        drawnow;
        pause(0.03); % Slow down animation
    end
end

% Final plot update after exploration
update_plot(ax, robot_map, visitedCells, N, hVisited, @update_discovered_lines);
update_robot_patch(hRobot, pose, 0.20);
drawnow;
disp('Exploration completed.');

%% ---------------- FLOOD FILL SHORTEST PATH ----------------
% PHASE 2: Calculate shortest path using the 'robot_map'
if goalFound
    % --- 1. Calculate shortest path ---
    path = floodfill_shortest_path(robot_map, startCell, goalCell, N);
    
    if isempty(path)
        title(ax, 'Goal was found but path is blocked!');
        disp('Error: Goal was found, but flood fill could not find a path back.');
    else
        % --- 2. Draw the calculated path ---
        title(ax, 'Shortest Path (Flood Fill)');
        px = (path(:,2)-0.5); % Path x-coordinates
        py = (N - path(:,1) + 0.5); % Path y-coordinates
        plot(ax, px, py, 'g-', 'LineWidth', 2.5);
        drawnow;
        
        %% --- 3. Reset Robot to Start and Follow the Path ---
        % PHASE 3: Execute the shortest path
        
        pose = [ (startCell(2)-1)+0.5; (N-startCell(1))+0.5; 0 ]; % Reset to start
        traj = pose';
        update_robot_patch(hRobot, pose, 0.20);
        drawnow;
        title(ax, 'Following Shortest Path');
        pause(1.0); % Pause to show path
        
        % --- Follow path cells smoothly ---
        for k = 2:size(path,1)
            center = cell_center_pose(path(k,:), N);
            
            % Drive to the center of the next cell in the path
            % using the *same* P-controller from exploration.
            while hypot(pose(1)-center(1), pose(2)-center(2)) > poseTol
                dx = center(1) - pose(1);
                dy = center(2) - pose(2);
                target_theta = atan2(dy, dx);
                e_theta = angdiff(target_theta, pose(3));
                rho = hypot(dx, dy);
                
                v = K_rho * rho * (abs(e_theta) < pi/2);
                omega = K_alpha * e_theta;
                
                v = max(min(v, max_v), -max_v);
                omega = max(min(omega, max_omega), -max_omega);
                
                pose(1) = pose(1) + v*cos(pose(3))*dt;
                pose(2) = pose(2) + v*sin(pose(3))*dt;
                pose(3) = wrapToPi(pose(3) + omega*dt);
                traj(end+1,:) = pose';
                
                update_robot_patch(hRobot, pose, 0.20);
                drawnow;
                pause(0.02);
            end
        end
        title(ax, 'Goal Reached via Flood Fill Shortest Path');
    end
else
    title(ax, 'Goal not reached in exploration');
end
disp('Simulation finished.');

%% ---------------- Helper Functions ----------------

function walls = sense_walls(maze, cell)  % returns 1*4 row vector [N E W S] for the specified cell
    % Simulates sensing: Peeks at the "ground truth" maze
    walls = squeeze(maze(cell(1), cell(2), :))';
end

function cpose = cell_center_pose(cell, N)
    % Converts discrete grid (row, col) to continuous (x, y)
    cpose = [(cell(2)-1)+0.5, (N-cell(1))+0.5];
end

function d = angdiff(a,b)
    % Calculates the smallest angle difference (e.g., -pi to +pi)
    d = mod(a - b + pi, 2*pi) - pi;
end

function update_robot_patch(hPatch, pose, size)
    % Updates the vertices of the robot's triangle patch
    theta = pose(3);
    pts = [ size,0; -size/2, size*sqrt(3)/2; -size/2,-size*sqrt(3)/2 ]; % Triangle shape
    R = [cos(theta), -sin(theta); sin(theta), cos(theta)]; % Rotation matrix
    pR = (R*pts')'; % Rotate points
    pR(:,1)=pR(:,1)+pose(1); % Translate X
    pR(:,2)=pR(:,2)+pose(2); % Translate Y
    set(hPatch,'XData',pR(:,1),'YData',pR(:,2));
end

function update_plot(ax, robot_map, visitedCells, N, hVisited, fn)
    % Colors visited cells
    for r=1:N
        for c=1:N
            if visitedCells(r,c)
                set(hVisited(r,c),'FaceColor',[0.8 0.9 1],'EdgeColor','none');
            else
                set(hVisited(r,c),'FaceColor','none');
            end
        end
    end
    % Calls the function to draw discovered walls
    fn(ax, robot_map, N);
end

function update_discovered_lines(ax, robot_map, N)
    % Draws walls from the robot's 'robot_map' memory in blue
    old=findall(ax,'Tag','discoveredWall'); if ~isempty(old), delete(old); end
    for r=1:N
        for c=1:N
            k = squeeze(robot_map(r,c,:)); % [N E S W]
            % Only draw known walls (k==1)
            if k(1)==1, line(ax, [c-1,c],[N-r+1,N-r+1],'Color','b','LineWidth',3,'Tag','discoveredWall'); end
    		if k(2)==1, line(ax, [c,c],[N-r,N-r+1],'Color','b','LineWidth',3,'Tag','discoveredWall'); end
    		if k(3)==1, line(ax, [c-1,c],[N-r,N-r],'Color','b','LineWidth',3,'Tag','discoveredWall'); end
    		if k(4)==1, line(ax, [c-1,c-1],[N-r,N-r+1],'Color','b','LineWidth',3,'Tag','discoveredWall'); end
        end
    end
end

function nbrs = accessible_unvisited_neighbors(robot_map, cell, visitedCells, N)
    % Helper for DFS: finds neighbors that are NOT blocked by a wall
    % (in robot_map) AND have not been visited yet.
    r=cell(1); c=cell(2); nbrs=[];
    w = squeeze(robot_map(r,c,:))'; % Get walls from robot's memory
    
    if w(1)==0 && r>1 && ~visitedCells(r-1,c), nbrs=[nbrs; r-1 c]; end % North
    if w(2)==0 && c<N && ~visitedCells(r,c+1), nbrs=[nbrs; r c+1]; end % East
    if w(3)==0 && r<N && ~visitedCells(r+1,c), nbrs=[nbrs; r+1 c]; end % South
    if w(4)==0 && c>1 && ~visitedCells(r,c-1), nbrs=[nbrs; r c-1]; end % West
end

% ---------- START: CORRECTED FUNCTION ----------
function path = floodfill_shortest_path(robot_map, startCell, goalCell, N)
    % Finds shortest path from START to GOAL using the ROBOT_MAP
    
    % --- 1. BUILD DISTANCE GRID (BFS from Goal) ---
    % Fills a grid 'dist' with the # of steps from each cell to the goal.
    dist = inf(N,N);
    q = goalCell; % Start search from the GOAL
    dist(goalCell(1),goalCell(2)) = 0;
    
    while ~isempty(q)
        cell = q(1,:); q(1,:) = []; % Dequeue
        r=cell(1); c=cell(2); d = dist(r,c)+1;
        
        % Check walls using robot's memory
        w = squeeze(robot_map(r,c,:))'; 
        
        % If no wall and path is shorter, update distance and enqueue neighbor
        if w(1)==0 && r>1 && d<dist(r-1,c), dist(r-1,c)=d; q=[q; r-1 c]; end
        if w(2)==0 && c<N && d<dist(r,c+1), dist(r,c+1)=d; q=[q; r c+1]; end
        if w(3)==0 && r<N && d<dist(r+1,c), dist(r+1,c)=d; q=[q; r+1 c]; end
        if w(4)==0 && c>1 && d<dist(r,c-1), dist(r,c-1)=d; q=[q; r c-1]; end
    end
    
    % Check if a path exists
    if isinf(dist(startCell(1),startCell(2))), path=[]; return; end
    
    % --- 2. RECONSTRUCT PATH (Gradient Descent from Start) ---
    % "Walk downhill" from Start to Goal on the 'dist' grid.
    cur=startCell; path=cur;
    
    while ~isequal(cur, goalCell)
        r=cur(1); c=cur(2); d=dist(r,c);
        
        % *** BUG FIX: Must check walls of *current* cell ***
        w = squeeze(robot_map(r,c,:))';
        
        nbrs=[r-1 c; r c+1; r+1 c; r c-1]; % [N, E, S, W]
        walls_to_check = [w(1); w(2); w(3); w(4)];
        
        valid=[];
        for i=1:4
            rr=nbrs(i,1); cc=nbrs(i,2);
            
            % A neighbor is valid if it's:
            % 1. In bounds
            % 2. "Downhill" (dist < d)
            % 3. NOT blocked by a wall (walls_to_check(i) == 0)
            if rr>=1 && rr<=N && cc>=1 && cc<=N && dist(rr,cc)<d && walls_to_check(i)==0
                valid=[valid; rr cc dist(rr,cc)];
            end
        end
        
        if isempty(valid)
            path=[]; % No valid path found
            break; 
        end
        
        % Move to the neighbor with the minimum distance
        [~,idx]=min(valid(:,3)); 
        cur=valid(idx,1:2); 
        path=[path; cur];
    end
end

function draw_true_maze(ax, maze, N)
    % Draws the light-grey "ground truth" maze at the start
    for r=1:N
        for c=1:N
            x0=c-1; y0=N-r; w=squeeze(maze(r,c,:))';
            if w(1), plot(ax,[x0 x0+1],[y0+1 y0+1],'Color',[0.7 0.7 0.7],'LineWidth',1); end
            if w(2), plot(ax,[x0+1 x0+1],[y0 y0+1],'Color',[0.7 0.7 0.7],'LineWidth',1); end
            if w(3), plot(ax,[x0 x0+1],[y0 y0],'Color',[0.7 0.7 0.7],'LineWidth',1); end
            if w(4), plot(ax,[x0 x0],[y0 y0+1],'Color',[0.7 0.7 0.7],'LineWidth',1); end
        end
    end
end