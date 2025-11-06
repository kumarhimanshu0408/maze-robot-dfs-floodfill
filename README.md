# Maze Robot Exploration & Path Planning (MATLAB)

This repository implements a wheeled robot that **explores an unknown maze**, builds its own internal map, calculates the **shortest path** using a **Flood-Fill** algorithm, and navigates using **continuous robot kinematics**.  
It also includes a **Maze Creator GUI** to design any **N×N** maze interactively.

---

## 🧱 Maze Creator GUI (`maze_creator.m`)

This project includes an **interactive Maze Creator GUI** that allows you to visually design **any N×N maze** by clicking to add or remove walls.  
The generated maze code can be **copied directly** into the main maze-exploration solver.

### ✅ Features
- Clickable wall segments (toggle ON/OFF)
- **Red = Wall present**, **Gray = No wall**
- Maze stored in format:  
  \[ \text{maze}(r,c,:) = [N\; E\; S\; W] \]
- Wall symmetry handled automatically (no broken cell edges)
- Outer boundary walls generated automatically
- Custom **Start** and **Goal** cells supported

---

### 🧩 How the GUI Works

| Function | Purpose |
|--------|---------|
| `setupAxes(ax, N)` | Initializes maze grid and coordinate system (row 1 at top). |
| `drawWalls(ax, N)` | Draws internal wall segments and registers them as clickable. |
| `toggleWall(src, ~)` | Toggles wall state visually and internally. |
| `generateMazeCode(h_walls, v_walls, N)` | Outputs full maze code in solver-compatible format. |

#### Wall Representation Format
```
maze(r,c,:) = [North  East  South  West];
1 = wall present
0 = open path
```

**Example Output:**
```matlab
maze(2,3,3)=1; maze(3,3,1)=1;
maze(4,1,2)=1; maze(4,2,4)=1;
```

---

### 🔗 Using GUI Output in the Solver

1. Run GUI:
```matlab
createMazeGUI;
```
2. Click to design maze.
3. Press **Generate Maze Code**.
4. Copy printed maze data.
5. Paste into the solver's **MAZE DEFINITION** section.
6. Adjust:
```matlab
startCell = [row, col];
goalCell  = [row, col];
```
7. Run the simulation.

---

## 🤖 Robot Kinematics & Motion Control

The robot follows the **unicycle kinematic model** with state:

\[
\mathbf{p} = \begin{bmatrix} x \\ y \\ \theta \end{bmatrix}
\]

### 🎯 Target Acquisition
\[
\rho = \sqrt{(x^* - x)^2 + (y^* - y)^2}
\]
\[
\theta^* = \text{atan2}(y^* - y, x^* - x)
\]
\[
\alpha = \text{wrapToPi}(\theta^* - \theta)
\]

### ⚙️ Control Law
\[
v = K_{\rho}\,\rho \, \mathbb{1}_{(|\alpha| < \pi/2)}
\]
\[
\omega = K_{\alpha}\,\alpha
\]

### 🚗 Motion Update
\[
x_{k+1} = x_k + v\cos(\theta_k)\,dt
\]
\[
y_{k+1} = y_k + v\sin(\theta_k)\,dt
\]
\[
\theta_{k+1} = \text{wrapToPi}(\theta_k + \omega\,dt)
\]

### Used In Both:
| Phase | Purpose |
|------|---------|
| Maze Exploration (DFS) | Discover walls and build internal `robot_map` |
| Shortest Path Following | Drive along flood-fill result |

---

## 🎯 Summary

| Component | Function |
|---------|----------|
| Maze Creator GUI | Create any N×N maze visually |
| DFS Exploration | Robot explores by sensing cell walls |
| `robot_map` | Stores discovered wall information |
| Flood-Fill Algorithm | Computes shortest path |
| Unicycle Kinematics | Produces smooth robot motion |

---


---

