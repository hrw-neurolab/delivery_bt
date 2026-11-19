# delivery_bt: the delivery mission as a behaviour tree with py_trees
---
Day 8 of the robotics course. The Python examples of slides 16 and 18 (`bt.py`, `mission.py`)
in ROS 2 with [py_trees](https://py-trees.readthedocs.io) and py_trees_ros: first the same tree
on the same line of cells, then the tree of slide 17 on top of Nav2 in the worlds of day 7.
The small simulator of `slam_nav` replaces Gazebo, now with a battery, a dock and a box that
can block a corridor. No GPU, starts in a few seconds.

```
                       delivery (py_trees) --navigate_to_pose--> Nav2 --/cmd_vel--> drive_sim
                        ^          |                                                  |  |  |
          /battery_state|          +--/dock/charge (service)--------------------------+  |  |
                        +--------------------------------------------------------------+  |
                                   amcl <--/scan-- laser_sim <--/sim/true_pose-------------+
```

| Node | Package | Job |
|---|---|---|
| `mission` | delivery_bt | the tree of slide 18 with py_trees, on the line of cells. No Nav2, no simulator |
| `delivery` | delivery_bt | the tree of slide 17 with py_trees: Nav2 goals, battery, dock |
| `drive_sim` | delivery_bt | true motion, drifting odometry, battery, dock (`/dock/charge`), block |
| `laser_sim` | delivery_bt | 2D lidar: rays from the true pose against the walls and the block, `/scan` |
| `robot_viz` | delivery_bt | true walls, stations, block, battery and robot, only for looking |
| `base_scan_tf` | tf2_ros | static tf `base_footprint -> base_scan` |
| `map_server`, `amcl` | Nav2 | the maps of day 7 (drawn with SLAM in `slam_nav`) and AMCL of day 6 |
| `controller_server`, `planner_server`, `behavior_server`, `bt_navigator` | Nav2 | navigation with the default tree of slide 22 |
| `lifecycle_manager_*` | nav2_lifecycle_manager | activate localization and navigation |

Two worlds, both in `delivery_bt/worlds.py`, each with three stations and a block:

* `loop`: the square corridor of day 7. Dock at the start, pickup 4 m and drop off 8 m along the
  corridor (the opposite corner), like `DOCK, PICKUP, DROPOFF = 0, 3, 8` of slide 18. The block
  closes the east corridor, Nav2 goes the other way round.
* `room`: the room of day 6 with the inner wall and the box. Pickup in the left half, drop off
  in the right half. The block closes the opening below the inner wall, the drop off cannot be
  reached any more.

---
## Setup

All commands use `$ROS_DISTRO`, so they work for any ROS 2 distro (Kilted, Lyrical, ...).
Sourcing ROS sets this variable. Check it in every new terminal:
```bash
echo $ROS_DISTRO
```
Empty? Set it to your distro (once per terminal, or add the line to `~/.bashrc`):
```bash
export ROS_DISTRO=lyrical   # or kilted, jazzy, ...
```

Create virtual Environment:
```bash
python3 -m venv .venv --system-site-packages
```
No extra pip packages are needed, everything comes from ROS.

Needed ROS packages (Nav2 is already there from day 6):
```bash
sudo apt install ros-$ROS_DISTRO-py-trees ros-$ROS_DISTRO-py-trees-ros ros-$ROS_DISTRO-navigation2
```
Optional, a window that draws the running tree (needs Qt WebEngine):
```bash
sudo apt install ros-$ROS_DISTRO-py-trees-ros-viewer
```

Switched to another distro? Its Python version is different too (Kilted 3.12, Lyrical 3.14),
so start clean and do Setup and Build again:
```bash
rm -rf .venv build install log
```

---
## Build
1. Source ROS:
```bash
source /opt/ros/$ROS_DISTRO/setup.bash
```
2. Build with:
```bash
source .venv/bin/activate
python3 -m colcon build
source install/setup.bash
```
Build again after changing any file, the nodes and launch files use the installed copy.

---
## 1. The mission of slide 18 with py_trees

```bash
ros2 run delivery_bt mission
```
The same tree, the same world, the same route as `mission.py`:
```
tick   1   pos 1   battery   65   RUNNING   go to pickup
...
tick   7   pos 5   battery   35   RUNNING   go to dock
...
tick  29   pos 0   battery  100   SUCCESS   go to dock
12345654321012345678765432100 SUCCESS 100
```

| `mission.py` (slide 18) | `delivery_bt/mission.py` |
|---|---|
| `w = dict(pos=0, battery=70, ...)` | the blackboard: `pos`, `battery`, `loaded`, `delivered` |
| `seq(...)`, `fb(...)` | `py_trees.composites.Sequence`, `Selector` with `memory=False` |
| `cond(lambda w: w["loaded"])` | `py_trees.behaviours.CheckBlackboardVariableValue` |
| `do(loaded=True)` | `py_trees.behaviours.SetBlackboardVariable` |
| `goto(PICKUP)` | class `GoTo`: `update()` moves one cell, `RUNNING` until there |
| `while status == R: mission(w)` | `py_trees_ros.trees.BehaviourTree.tick_tock()`, a ROS timer |

See every node of the tree after each tick (slide 15). `[*]` running, `[✓]` success,
`[✕]` failure, `[-]` not ticked:
```bash
ros2 run delivery_bt mission --ros-args -p show_tree:=true
```
```
tick   7   pos 5   battery   35   RUNNING   go to dock
[-] mission [*]
    [o] battery [*]
        --> battery above 40? [✕] -- 'battery' comparison failed [v: 40.0][e: 40.0]
        [-] recharge [*]
            --> go to dock [*] -- pos 5
            --> charge [-]
    [o] pickup [-]
...
```
`[-]` in front of a name is a Sequence, `[o]` a Selector (Fallback), `-->` a leaf.

---
## 2. The mission of slide 17 on top of Nav2

Start the world: simulator, map, AMCL, Nav2 and RViz.
```bash
ros2 launch delivery_bt delivery.launch.py                 # loop
ros2 launch delivery_bt delivery.launch.py world:=room     # room of day 6
```
The robot starts on the dock, which is the origin of the map. No 2D Pose Estimate needed.

Run the tree in a second terminal and source both setups there too:
```bash
source /opt/ros/$ROS_DISTRO/setup.bash
source install/setup.bash
```
```bash
ros2 run delivery_bt delivery                              # loop
ros2 run delivery_bt delivery --ros-args -p world:=room    # room, same world as the launch
```
The robot drives to the pickup, waits 3 s for the box, drives towards the drop off, the battery
runs low on the way, it turns back to the dock, charges to 100 %, drives to the drop off without
visiting the pickup again (the box is still on top), unloads and returns to the dock. One log
line each time the running leaf changes, and the whole tree with the status of every node:
```
battery  70 %   RUNNING  go to pickup (sent goal request)
battery  54 %   RUNNING  load (still running)
battery  54 %   RUNNING  go to drop off (sent goal request)
battery  40 %   RUNNING  go to dock (sent goal request)
battery   9 %   RUNNING  charge
battery 100 %   RUNNING  go to drop off (sent goal request)
battery  68 %   RUNNING  unload (still running)
battery  68 %   RUNNING  go to dock (sent goal request)
battery  38 %   RUNNING  charge (100 %)
battery 100 %   SUCCESS  go to dock (successfully completed)
Mission SUCCESS, battery 100 %, loaded True, delivered True.
```
Like `mission.py`, the robot charges once more at the end: it comes home below 40 %.

The tree, with the names that appear in the log:
```
{-} delivery                      Sequence, memory=False: starts left on every tick
    --> battery to blackboard     data gathering: /battery_state -> battery, charging
    [o] battery                   Selector (Fallback), memory=False
        --> battery above 40?     condition
        {-} recharge              Sequence, memory=True: continues at the running child
            --> go to dock        Nav2 goal, RUNNING while Nav2 drives
            --> charge            service /dock/charge, RUNNING until the battery is full
    [o] pickup
        --> loaded?
        {-} fetch
            --> go to pickup
            --> load              3 s, a person puts the box on top
            --> loaded = True
    [o] dropoff
        --> delivered?
        {-} bring
            --> go to drop off
            --> unload
            --> delivered = True
    --> go to dock
```

| `mission.py` (slide 18) | `delivery_bt/delivery.py` |
|---|---|
| `w["battery"]` | blackboard `battery`, written by `battery to blackboard` from `/battery_state` |
| `w["loaded"]`, `w["delivered"]` | blackboard `loaded`, `delivered` |
| `goto(PICKUP)` | `go_to()`: action client of Nav2's `navigate_to_pose` (`py_trees_ros.action_clients.FromConstant`) |
| `do(battery=100)` | class `Charge`: plug in with `/dock/charge`, `RUNNING` until the battery is full |
| `do(loaded=True)` | a `Timer` of 3 s, then `SetBlackboardVariable` |

Three differences to slide 17, all on purpose:

* **memory=True in the action sequences** (slide 19). The root and the selectors are reactive
  (`memory=False`), so the battery is checked on every tick and can interrupt everything. The
  sequences of actions below them (`recharge`, `fetch`, `bring`) continue at the running child.
  On the line of slide 18 `goto` answers SUCCESS at once when the robot is there, so memory does
  not matter. A Nav2 goal takes time, see experiment 4.
* **The battery is not ok while charging.** `battery above 40?` alone would answer SUCCESS at
  41 % and stop the charging. The robot would leave the dock and come straight back. So the
  condition also reads `charging` from the battery: on the dock it waits until the battery is full.
* **Data gathering on the left.** `battery to blackboard` copies the latest message of
  `/battery_state` to the blackboard on every tick, the usual pattern of py_trees_ros. The
  conditions only read the blackboard.

A halted `go to ...` cancels its Nav2 goal, like the halted D of slide 15. When the battery
branch interrupts a drive, the new goal of `go to dock` replaces the old one in Nav2
(log of bt_navigator: `Received goal preemption request`). Ctrl+C halts the whole tree,
the robot stops.

### Watch the tree and the blackboard

In a third terminal (both setups sourced) while `delivery` or `mission` runs:
```bash
py-trees-tree-watcher
py-trees-blackboard-watcher
```
With the viewer installed: `py-trees-tree-viewer`. For `mission`, tick slowly so you can follow
it: `ros2 run delivery_bt mission --ros-args -p period:=1.0`.

### Charge and the battery by hand

```bash
ros2 topic echo /battery_state --field percentage          # 0 to 1, like every ROS battery
ros2 service call /dock/charge std_srvs/srv/Trigger        # only works on the dock
ros2 param set /drive_sim battery 35.0                     # battery low, right now
```
At 0 % `drive_sim` stops the robot: `ros2 param set /drive_sim battery 100.0`.

---
## In RViz

| What | Meaning |
|---|---|
| orange square | the dock: start and charger |
| blue squares | pickup and drop off |
| text above the world | battery in %, `charging` on the dock, orange at 40 % and below |
| orange box | the block, when it is switched on |
| grey robot | where the robot really is (only the simulator knows this) |
| blue robot | where the robot believes it is: tf `map -> odom -> base_footprint` |
| blue beams | the scan, drawn from the believed pose |
| thin dark lines | the true walls. The map should lie exactly under them |
| orange line | the global plan of NavFn |
| coloured square | the local costmap around the robot |
| Particles (AMCL) | the particle cloud of AMCL |

---
## Experiments for the exercise (slide 28)

1. **Move the threshold.** On the line of cells first:
   ```bash
   ros2 run delivery_bt mission --ros-args -p threshold:=30.0
   ```
   Where does the robot turn, and what does the battery show on the way back? Then on Nav2:
   `ros2 run delivery_bt delivery --ros-args -p threshold:=30.0`. Letting it depend on the distance
   (`w["battery"] > 5 * w["pos"] + 10`): in `create_mission()` of `mission.py` the value of a
   `cond(...)` may be a function without arguments, py_trees calls it on every tick.
2. **Battery low at any moment** (slide 8). While the robot loads or drives to the drop off:
   ```bash
   ros2 param set /drive_sim battery 35.0
   ```
   The battery branch takes over on the very next tick (slide 15). After charging the robot
   continues where it was. Count the edits the state machine of slide 6 would need (slide 8).
3. **Block the drop off** (slides 24 and 28). While the robot drives, or from the start with
   `blocked:=true` on the launch file:
   ```bash
   ros2 param set /drive_sim blocked true
   ```
   In the loop Nav2 plans the other way round as soon as the lidar sees the block. The box is not
   in the map, so a path planned before may still lead to it: then the controller gets stuck in
   front of it ("Failed to make progress"), a recovery runs and the next plan goes round. The
   mission still ends with SUCCESS. In the room there is no way to the drop off: Nav2 runs its
   recoveries (spin, wait, back up), gives up, `go to drop off` answers FAILURE and with it the
   whole mission. The state machine of slide 6 drives to the dock
   after a failure. Add a Fallback at the root so that the tree does the same.
   `ros2 param set /drive_sim blocked false` takes the block out again.
4. **Memory** (slide 19). Start the tree with
   ```bash
   ros2 run delivery_bt delivery --ros-args -p memory:=false
   ```
   At the pickup the finished `go to pickup` starts again on every tick, sends a new goal and
   halts `load`. The robot never gets loaded.
5. **The state machine of slide 26** runs against this simulator as it is. AMCL starts at the
   dock, so `waitUntilNav2Active()` sets the right pose. The stations in frame map:

   | world | dock | pickup | drop off |
   |---|---|---|---|
   | `loop` | (0.0, 0.0) | (4.0, 0.0) | (4.0, -4.0) |
   | `room` | (0.0, 0.0) | (-0.7, 1.5) | (3.9, 1.3) |

   It knows nothing about the battery. Compare with the tree.
6. **Nav2's own tree** (slide 25). Copy `navigate_to_pose_w_replanning_and_recovery.xml`, set
   `number_of_retries="2"` at the root, put the path into `default_nav_to_pose_bt_xml` of
   `bt_navigator` in `config/nav2.yaml`, build again, and repeat experiment 3 in the room.

---
## Parameters

`delivery`: `world` (loop), `threshold` (40 %), `load_time` (3 s), `memory` (true), `period`
(0.1 s), `show_tree` (true).

`mission`: `threshold` (40), `battery` (70), `period` (0.1 s), `show_tree` (false).

`drive_sim`: as in `slam_nav`, plus `battery` (70 %, set it while running to jump),
`battery_drain` (% per metre, loop 4, room 9), `charge_rate` (10 %/s), `dock_radius` (0.35 m),
`blocked` (false).

Launch arguments of `delivery.launch.py`: `world` (loop), `map` (`maps/<world>.yaml`), `battery`
(70), `blocked` (false), `rviz` (true).

The loop uses 4 % per metre, not the 5 per cell of slide 16: the real way back is about 1.2 m
longer than the way out (turning round, corners), and with 5 % only 4 % were left at the dock.
The maps in `maps/` and the files `config/nav2.yaml` and `config/amcl.yaml` are the ones of
`slam_nav` (day 7).
