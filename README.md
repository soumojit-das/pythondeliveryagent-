Autonomous Delivery Agent 

This project features an autonomous agent designed to navigate a 2D grid-based city to find the most efficient delivery routes. It implements and compares various pathfinding algorithms, handling static obstacles, variable terrain costs, and dynamic obstacles that require real-time replanning.

This is a solution for the "Fundamentals of AI and ML" (CSA2001) project.

-----
 Features 

  * **Grid Environment**: Simulates a city grid with walls, different terrain costs, and dynamic obstacles.
  * **Uninformed Search**: Implements **Uniform-Cost Search (UCS)** to find the cheapest path without any guidance.
  * **Informed Search**: Implements **A\* Search** using the Manhattan distance heuristic for a much more efficient search.
  * **Dynamic Replanning**: Demonstrates the agent's ability to adapt to new obstacles by recalculating its path mid-journey.
  * **Performance Analysis**: The script outputs key metrics like path cost, nodes expanded, and execution time to compare algorithm efficiency.
  * **Command-Line Interface**: Easy-to-use CLI to run simulations with different maps and algorithms.

-----

Installation 

No external libraries are required to run this project. You just need Python 3 installed on your system.

1.  Clone the repository to your local machine:
    ```bash
    git clone https://github.com/your-username/autonomous-delivery-agent.git
    ```
2.  Navigate to the project directory:
    ```bash
    cd autonomous-delivery-agent
    ```

-----

## How to Run 🚀

You can run the simulation from your terminal. The script requires two arguments: a **map name** and an **algorithm name**.

**Command Format:**

```bash
python delivery_agent.py <map_name> <algorithm>
```

### **Available Maps:**

  * `small`
  * `medium`
  * `large`
  * `dynamic` (designed for the replanning demo)

### **Available Algorithms:**

  * `ucs` (Uniform-Cost Search)
  * `astar` (A\* Search)
  * `replan_demo` (Demonstrates dynamic replanning)

### **Example Commands:**

**1. Run A\* search on the `medium` map:**

```bash
python delivery_agent.py medium astar
```

**2. Run Uniform-Cost Search on the `small` map:**

```bash
python delivery_agent.py small ucs
```

**3. Run the dynamic replanning demonstration:**

```bash
python delivery_agent.py dynamic replan_demo
```

This will show the agent finding an initial path, encountering a new obstacle, and then successfully finding a new route to the goal.

-----

## Map File Format 🗺️

The maps are simple `.txt` files where each character represents a grid cell:

  * `S`: The starting position of the agent.
  * `G`: The goal (delivery destination).
  * `X`: A static, impassable obstacle (a wall).
  * `1`, `2`, `3`, etc.: A walkable tile with the given integer movement cost, which is always 1 or greater.
