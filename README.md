# This forks

1. appended which human is visible or not
2. appended Pure-Pursuit (reference code: [repo](https://github.com/TempleRAIL/drl_vo_nav) )


### 1. Add which agent is visibility or not ###

<p align="center">
<img src="https://github.com/user-attachments/assets/a1bfef5c-02d9-4d1b-b0cd-820a6395f164"  width="300" height="300"/>
</p>

e.g. (in CrowdNav[https://github.com/vita-epfl/CrowdNav])

```
class MYClass():
    def __init__(self):
        pass

    def sort_humans(
        self,
        robot: Robot,
        humans: List[Human]
        ):
                ## sorting human by descending_order for make Occupancy 
        distances = []
        for human in humans:
            dist = np.linalg.norm([human.px- robot.px, human.py - robot.py])
            distances.append(dist)
        descending_order = np.array(distances).argsort()
        humans = np.array(humans)[descending_order]
        humans = humans.tolist()
        return humans 

    def _cb_lidar(
        self, 
        robot: Robot, 
        humans: List[Human,], 
        flat_contours: np.ndarray,
        distances_travelled_in_base_frame: np.ndarray
        ) -> Tuple[np.ndarray, np.ndarray, np.ndarray, List[bool]]:
        humans = self.sort_humans(robot,humans)
        lidar_pos = np.array([robot.px, robot.py, robot.theta], dtype=np.float32)
        ranges = np.ones((self.n_angles,), dtype=np.float32) * self.max_range
        angles = np.linspace(self.scan_min_angle,
                             self.scan_max_angle-self.scan_increment,
                             self.n_angles) + lidar_pos[2]
        other_agents = []
        for i, human in enumerate(humans):
            pos = np.array([human.px, human.py, human.theta], dtype=np.float32)
            dist = distances_travelled_in_base_frame[i].astype(np.float32)
            vel = np.array([human.vx, human.vy], dtype=np.float32)
            if self.lidar_legs:
                agent = CSimAgent(pos, dist, vel, type_="legs", radius = self.leg_radius)
            else:
                agent = CSimAgent(pos, dist, vel, type_="trunk", radius=human.radius)
            other_agents.append(agent)

        self.converter_cmap2d.render_agents_in_lidar(ranges, angles, other_agents, lidar_pos[:2])

        which_visible = [agent.get_agent_which_visible() for agent in other_agents] or which_visible = [agent.visible for agent in other_agent]
        return ranges, angles, distances_travelled_in_base_frame, which_visible

```

#### Notion #### 

```get_agent_which_visible``` or ```visible``` must be called after render_agents_in_lidar

```sort_humans``` must be used, before input humans into ```render_agents_in_lidar``` (i recommend that you should use ```sort_humans``` human step and human generate function)


### 2. Add subgoal planner (Called Pure-Pursuit) ###


```
class MYClass():
    def __init__(self):
        self.look_ahead_planner = LookAheadPlanner(look_ahead_dist = 2.5)

    def _cb_subgoal(self, robot: Robot, path:np.ndarray):
        path = path.astype(np.float32)
        position = np.array([robot.px, robot.py], dtype = np.float32)
        sub_goal = self.look_ahead_planner.find_subgoal(path, position)
        return sub_goal 
    

```



# pymap2d

pymap2d is a Cython-based fast toolbox for 2d grid maps.

The CMap2D class provides:
- simple xy <-> ij coordinate conversions
- implementation of the dijkstra / fastmarch algorithm
- fast 2D distance transform (ESDF)
- conversions:
  - to/from polygon vertices
  - from ROS occupancy map or lidar scan message
  - serialization to/from dict

![pymap2d](media/pymap2d.png)

Note: rather than carefully designed, this codebase was chaotically grown. 
It is in dire need of refactoring / documentation. I hope it still proves useful.

## Dependency: Cython
```
$ pip3 install numpy==1.23.4 Cython==0.29.37
```

## Installation:
Inside this project root folder:
```
$ pip3 install -e .
```

## How to

Creating a map

```python
from CMap2D import CMap2D

# empty map
mymap = CMap2D()

# from an array
mymap.from_array(array, origin, resolution)

# from a pgm file
mymap = CMap2D("folder", "filename")

# from a ROS message
mymap.from_msg(msg)
```

Accessing occupancy data, origin, resolution (read-only)

```python
# occupancy as 2d array
mymap.occupancy()

# origin: (x, y) coordinates of point (i, j) = (0, 0)
mymap.origin_xy()

# resolution: size of grid cell [meters]
mymap.resolution()
```

Converting between grid and spatial coordinates

```python
list_of_xy_points = np.array([[1.3, 2.3], [-1.1, -4.0], [6.4, 2.3]])

in_ij_coordinates = mymap.xy_to_floatij(list_of_xy_points)
as_indices = mymap.xy_to_ij(list_of_xy_points, clip_if_outside=True)
```

gridshow is a convenience function, which wraps plt.imshow to intuitively visualize 2d array contents.
It makes the first array dimension x axis and uses grayscale by default.

```python
from CMap2D import gridshow
gridshow(mymap.occupancy())
```

![gridshow_vs_imshow](media/gridshow_vs_imshow.png)


For more examples, see [test](test)/example_*.py
