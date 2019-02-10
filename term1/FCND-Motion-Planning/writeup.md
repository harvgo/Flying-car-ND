## Project: 3D Motion Planning

# Required Steps for a Passing Submission:
1. Load the 2.5D map in the colliders.csv file describing the environment.
2. Discretize the environment into a grid or graph representation.
3. Define the start and goal locations.
4. Perform a search using A* or other search algorithm.
5. Use a collinearity test or ray tracing method (like Bresenham) to remove unnecessary waypoints.
6. Return waypoints in local ECEF coordinates (format for `self.all_waypoints` is [N, E, altitude, heading], where the drone’s start location corresponds to [0, 0, 0, 0].
7. Write it up.
8. Congratulations!  Your Done!

## [Rubric](https://review.udacity.com/#!/rubrics/1534/view) Points

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it! Below I describe how I addressed each rubric point and where in my code each point is handled.

### Explain the Starter Code

#### 1. Explain the functionality of what's provided in `motion_planning.py` and `planning_utils.py`
These scripts contain a basic planning implementation based on the backyard_flyer project. In `motion_planninng.py`, includes an additional state, PLANNING, where the system calculate the necessary waypoints from start to goal position.

In `planning_utils.py` are defined all the functions required for the planning to be executed:

- `create_grid` which represents the obstacles to avoid by the drone in 2.5D format.
- `Action` where the drone's possible movements are indicated.
- `valid_actions` indicates the boundaries of the grid.
- `a_star` returns the path (if any), with the lowest cost from start to goal.
- `heuristic`, required by `a_star`, and provides a metric to calculate the path cost.
- `collinearity_prune` helps to reduce the number of nodes in the path, in order to make it smoother.

### Implementing Your Path Planning Algorithm

#### 1. Set your global home position
In the line 124, the collision data is open and and the first file line is read. Then, the conversion from string to float is necessary in order to obtain the initial global coordinates, and set them as the home position.

#### 2. Set your current local position
From line 133 to 139, the global position should be retrieved and then converted to local coordinates.

#### 3. Set grid start position from local position
Starting in line 143, all the obstacles are arranged in a grid in 2.5D format (width and length plus height as a value). Afterwards,
the start position of the quadrotor is settled based on the local position using:

```python
grid_start = (int(np.ceil(local_position[0] - north_offset)), int(np.ceil(local_position[1] - east_offset)))
```

#### 4. Set grid goal position from geodetic coords
Although in the code is required to set an arbitrary position in latitude and longitude coordinates, I decided to randomize the procedure using the boundaries of the grid (Lines 155-161). So, any time the script runs, a new goal position is settled. I based this procedure on the `colliders.csv` coordinates format.

#### 5. Modify A* to include diagonal motion (or replace A* altogether)
Here, the script `planning_utils.py` was modified, specifically the class `Action`. There, new maneuvers have been added (NE, NW, SE, SW) with the correspondent cost of 1.4142 (`sqrt(2)`).

Additionally, the validation of those actions (`valid_actions(grid, current_node)`) is performed in order to fall within the grid boundaries and out of obstacles (lines 44-100)

#### 6. Cull waypoints 
An additional function (`collinearity_prune(path, epsilon=1e-6)`) was included in the script (line 157). It is mainly based on the version given in the lectures, where the collinearity of three nodes is checked based on the determinant calculation of three nodes concatenated as a matrix. In that way, it is possible to reduce the number of nodes to be visited.



### Execute the flight
#### 1. Does it work?
The simulator displays the waypoints and the quadrotor flies following the route traced by them.
