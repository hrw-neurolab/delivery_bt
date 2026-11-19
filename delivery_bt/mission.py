"""
The mission of slide 18 with py_trees: the same tree, the same world, the same route.

No Nav2, no simulator: the world is the line of slide 18, one cell per tick.
Compare it line by line with bt.py and mission.py of slides 16 and 18:

    mission.py (slide 18)               this node
    ---------------------------------   -----------------------------------------------
    w = dict(pos=0, battery=70, ...)    the blackboard: pos, battery, loaded, delivered
    seq(...), fb(...)                   py_trees.composites.Sequence and Selector,
                                        memory=False: every tick starts at the left
    cond(lambda w: w["loaded"])         py_trees.behaviours.CheckBlackboardVariableValue
    do(loaded=True)                     py_trees.behaviours.SetBlackboardVariable
    goto(PICKUP)                        GoTo below: one cell per tick, RUNNING until there
    while status == R: mission(w)       py_trees_ros.trees.BehaviourTree.tick_tock,
                                        a ROS timer ticks the tree

Run:
    ros2 run delivery_bt mission
prints one line per tick and at the end the output of slide 18:
    12345654321012345678765432100 SUCCESS 100

Exercise of slide 28, move the threshold:
    ros2 run delivery_bt mission --ros-args -p threshold:=30.0
Watch the tree while it runs, slowly, in a second terminal:
    ros2 run delivery_bt mission --ros-args -p period:=1.0
    py-trees-tree-watcher

ROS parameters
    threshold   battery in %, "battery above 40?" of slide 17 (default 40)
    battery     battery at the start (default 70)
    period      s between two ticks (default 0.1)
    show_tree   true: print the whole tree with the status of every node after each tick
"""
import operator
from typing import Any

import py_trees
from py_trees.common import Access, ComparisonExpression, Status
import py_trees_ros
import rclpy

DOCK, PICKUP, DROPOFF = 0, 3, 8   # the cells of slide 18


# --- the leaves of slide 16 -----------------------------------------------------

def cond(name: str, variable: str, op: Any, value: Any) -> py_trees.behaviour.Behaviour:
    """Compare a blackboard variable, a condition: SUCCESS or FAILURE, never RUNNING."""
    return py_trees.behaviours.CheckBlackboardVariableValue(
        name=name, check=ComparisonExpression(variable=variable, value=value, operator=op))


def do(name: str, variable: str, value: Any) -> py_trees.behaviour.Behaviour:
    """Change the world (the blackboard), an instant action that answers SUCCESS."""
    return py_trees.behaviours.SetBlackboardVariable(
        name=name, variable_name=variable, variable_value=value, overwrite=True)


class GoTo(py_trees.behaviour.Behaviour):
    """Action goto(x) of slide 16: one cell per tick, RUNNING until there."""

    def __init__(self, name: str, x: int) -> None:
        super().__init__(name=name)
        self.x = x
        # a client of the blackboard, it may read and write pos and battery
        self.bb = self.attach_blackboard_client(name=name)
        self.bb.register_key(key='pos', access=Access.WRITE)
        self.bb.register_key(key='battery', access=Access.WRITE)

    def update(self) -> Status:
        if self.bb.pos == self.x:
            return Status.SUCCESS
        self.bb.pos += 1 if self.x > self.bb.pos else -1
        self.bb.battery -= 5
        self.feedback_message = f'pos {self.bb.pos}'
        return Status.RUNNING


# --- the tree of slide 17 ---------------------------------------------------------

def seq(name: str, *children: py_trees.behaviour.Behaviour) -> py_trees.composites.Sequence:
    return py_trees.composites.Sequence(name=name, memory=False, children=list(children))


def fb(name: str, *children: py_trees.behaviour.Behaviour) -> py_trees.composites.Selector:
    return py_trees.composites.Selector(name=name, memory=False, children=list(children))


def create_mission(threshold: float) -> py_trees.behaviour.Behaviour:
    """Build the tree of slide 18, line by line. py_trees calls a Fallback a Selector."""
    return seq('mission',
               fb('battery',
                  cond(f'battery above {threshold:.0f}?', 'battery', operator.gt, threshold),
                  seq('recharge', GoTo('go to dock', DOCK), do('charge', 'battery', 100))),
               fb('pickup',
                  cond('loaded?', 'loaded', operator.eq, True),
                  seq('fetch', GoTo('go to pickup', PICKUP), do('load', 'loaded', True))),
               fb('dropoff',
                  cond('delivered?', 'delivered', operator.eq, True),
                  seq('bring', GoTo('go to drop off', DROPOFF),
                      do('unload', 'delivered', True))),
               GoTo('go to dock', DOCK))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = rclpy.create_node('mission')
    threshold: float = node.declare_parameter('threshold', 40.0).value
    battery: float = node.declare_parameter('battery', 70.0).value
    period: float = node.declare_parameter('period', 0.1).value
    show_tree: bool = node.declare_parameter('show_tree', False).value

    # w = dict(pos=0, battery=70, loaded=False, delivered=False)
    w = py_trees.blackboard.Client(name='main')
    for key in ('pos', 'battery', 'loaded', 'delivered'):
        w.register_key(key=key, access=Access.WRITE)
    w.pos, w.battery, w.loaded, w.delivered = DOCK, battery, False, False

    tree = py_trees_ros.trees.BehaviourTree(create_mission(threshold))
    tree.setup(node=node, timeout=5.0)
    print(py_trees.display.unicode_tree(tree.root), flush=True)

    route = ''
    done = False

    def after_tick(tree: py_trees_ros.trees.BehaviourTree) -> None:
        """Print the tick, like route += str(w["pos"]) on slide 18."""
        nonlocal route, done
        route += str(w.pos)
        tip = tree.root.tip()
        print(f'tick {tree.count + 1:3d}   pos {w.pos}   battery {w.battery:4.0f}   '
              f'{tree.root.status.value:8}  {tip.name if tip else ""}', flush=True)
        if show_tree:
            print(py_trees.display.unicode_tree(tree.root, show_status=True), flush=True)
        if tree.root.status != Status.RUNNING:   # while status == R
            done = True

    tree.add_post_tick_handler(after_tick)
    tree.tick_tock(period_ms=1000.0 * period)
    try:
        while rclpy.ok() and not done:
            rclpy.spin_once(node, timeout_sec=0.1)
        print(route, tree.root.status.value, f'{w.battery:.0f}', flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        tree.shutdown()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
