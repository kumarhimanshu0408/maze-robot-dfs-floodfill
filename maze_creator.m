function createMazeGUI()
    % Creates a 6x6 GUI to define a maze by clicking on walls.
    % The generated code is printed to the Command Window.
    
    N = 6; % Maze size (6x6)
    
    % --- Create Figure and Axes ---
    fig = uifigure('Name', 'Maze Creator (6x6)', 'Position', [100 100 560 620]);
    ax = uiaxes(fig, 'Position', [20 80 520 520]);
    
    % --- Configure Axes ---
    setupAxes(ax, N);
    
    % --- Draw Clickable Internal Walls ---
    % **FIX:** 'drawWalls' now returns the handles to the created wall lines.
    [h_walls, v_walls] = drawWalls(ax, N);
    
    % --- Add "Generate Code" Button ---
    % This button callback now correctly uses the 'h_walls' and 'v_walls'
    % variables returned from the 'drawWalls' function.
    uibutton(fig, 'push', ...
           'Text', 'Generate Maze Code (to Command Window)', ...
           'Position', [150 20 260 30], ...
           'ButtonPushedFcn', @(src,event) generateMazeCode(h_walls, v_walls, N));

end % main function

% -----------------
% HELPER FUNCTIONS
% -----------------

function setupAxes(ax, N)
    % Configures the axes for the maze grid
    ax.XLim = [0.5 N+0.5];
    ax.YLim = [0.5 N+0.5];
    ax.YDir = 'reverse'; % Set (1,1) to be top-left
    ax.DataAspectRatio = [1 1 1]; % Ensure cells are square
    ax.XTick = 1:N;
    ax.YTick = 1:N;
    ax.Box = 'on'; % This draws the outer boundary wall
    title(ax, 'Click on gray lines to add/remove walls');
    hold(ax, 'on'); % Keep axes "on" for drawing walls
end

% **FIX:** Changed function to return the two handle arrays
function [h_walls, v_walls] = drawWalls(ax, N)
    % Draws the clickable horizontal and vertical wall lines
    
    % **FIX:** Pre-allocation arrays are now created *inside* this function
    h_walls = gobjects(N-1, N); % Horizontal walls
    v_walls = gobjects(N, N-1); % Vertical walls
    
    % Draw horizontal walls (between rows)
    for r = 1:N-1
        for c = 1:N
            % A line from (c-0.5, r+0.5) to (c+0.5, r+0.5)
            h_walls(r,c) = line(ax, [c-0.5 c+0.5], [r+0.5 r+0.5], ...
                'Color', [0.8 0.8 0.8], 'LineWidth', 4, ...
                'UserData', 0, 'ButtonDownFcn', @toggleWall); % UserData: 0=off, 1=on
        end
    end
    
    % Draw vertical walls (between columns)
    for r = 1:N
        for c = 1:N-1
            % A line from (c+0.5, r-0.5) to (c+0.5, r+0.5)
            v_walls(r,c) = line(ax, [c+0.5 c+0.5], [r-0.5 r+0.5], ...
                'Color', [0.8 0.8 0.8], 'LineWidth', 4, ...
                'UserData', 0, 'ButtonDownFcn', @toggleWall); % UserData: 0=off, 1=on
        end
    end
    hold(ax, 'off');
end

function toggleWall(src, ~)
    % Callback function for clicking on a wall
    if src.UserData == 0 % Wall is OFF
        src.UserData = 1; % Turn ON
        src.Color = 'r';  % Make it red
        src.LineWidth = 3;
    else % Wall is ON
        src.UserData = 0; % Turn OFF
        src.Color = [0.8 0.8 0.8]; % Make it gray
        src.LineWidth = 4;
    end
end

function generateMazeCode(h_walls, v_walls, N)
    % Prints the maze definition code to the Command Window
    
    % --- Print Header ---
    fprintf('\n\n');
    fprintf('%% --- MAZE DEFINITION (Generated) ---\n');
    fprintf('%% This is the "Ground Truth" map, not visible to the robot.\n');
    fprintf('%% maze(r,c,:) = [N E S W], 1 = wall, 0 = open\n');
    fprintf('N = %d;\n', N);
    fprintf('maze = zeros(N,N,4);\n\n');

    % --- Print Outer walls ---
    fprintf('%% --- Outer walls ---\n');
    fprintf('maze(:,1,4)=1; maze(:,N,2)=1; %% West, East\n');
    fprintf('maze(1,:,1)=1; maze(N,:,3)=1; %% North, South\n\n');

    % --- Print Internal walls ---
    fprintf('%% --- Internal walls ---\n');
    fprintf('%% Each pair of lines defines one wall.\n');
    
    wallCount = 0;
    
    % Iterate through horizontal walls
    [rows, cols] = size(h_walls);
    for r = 1:rows
        for c = 1:cols
            % This 'if' statement will no longer error
            if h_walls(r,c).UserData == 1
                % Horizontal wall between (r,c) and (r+1,c)
                % Sets South wall of (r,c) and North wall of (r+1,c)
                fprintf('maze(%d,%d,3)=1; maze(%d,%d,1)=1;\n', r, c, r+1, c);
                wallCount = wallCount + 1;
            end
        end
    end
    
    % Iterate through vertical walls
    [rows, cols] = size(v_walls);
    for r = 1:rows
        for c = 1:cols
            if v_walls(r,c).UserData == 1
                % Vertical wall between (r,c) and (r,c+1)
                % Sets East wall of (r,c) and West wall of (r,c+1)
                fprintf('maze(%d,%d,2)=1; maze(%d,%d,4)=1;\n', r, c, r, c+1);
                wallCount = wallCount + 1;
            end
        end
    end

    if wallCount == 0
        fprintf('%% (No internal walls were added)\n');
    end

    % --- Print Footer ---
    fprintf('%% ------------------------------------------------------------\n');
    % Note: These are hardcoded as per your example.
    fprintf('startCell = [1,1];   %% Robot starts at (row 1, col 1)\n');
    fprintf('goalCell  = [6,4];   %% The target is (row 6, col 4)\n');
    fprintf('%% --- End of Generated Code ---\n\n');
    
    disp('Maze code printed to Command Window.');
end