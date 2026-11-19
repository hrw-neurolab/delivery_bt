"""
The delivery mission of slide 17 as a behaviour tree with py_trees, on top of Nav2.

The same tree as mission.py of slide 18, now in the world of day 7: the
leaves send Nav2 goals, the battery comes from the simulator, the world
dict w is the blackboard of the tree.

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

    mission.py (slide 18)              this node
    ---------------------------------  ------------------------------------------------
    w["battery"]                       blackboard battery, written by "battery to
                                       blackboard" from the topic /battery_state
    w["loaded"], w["delivered"]        blackboard loaded, delivered
    goto(PICKUP)                       GoTo: action client of Nav2's navigate_to_pose
                                       (py_trees_ros.action_clients.FromConstant)
    do(battery=100)                    Charge: plug in, wait until the battery is full
    do(loaded=True)                    a Timer of 3 s, then SetBlackboardVariable
    seq, fb                            Sequence and Selector of py_trees

Why memory=True in recharge, fetch and bring? Slide 19: the root and the
selectors are reactive (memory=False), so the battery is checked on every
tick and can interrupt everything. The sequences of actions below them
continue at the running child (memory=True). With memory=False a finished
"go to pickup" would send a new goal on every tick and halt "load" again
and again. Try it: -p memory:=false

Why is the battery not ok while charging? "battery above 40?" alone would
answer SUCCESS at 41 % and stop the charging, the robot would leave the dock
and come back at once. So the condition also asks the dock: while the robot
is charging, the battery counts as low, until it is full.

Run, after the world (first terminal):
    ros2 launch delivery_bt delivery.launch.py
    ros2 run delivery_bt delivery
In the room of day 6:
    ros2 launch delivery_bt delivery.launch.py world:=room
    ros2 run delivery_bt delivery --ros-args -p world:=room
Watch the tree and the blackboard while it runs (third terminal):
    py-trees-tree-watcher
    py-trees-blackboard-watcher

ROS parameters
    world        loop or room, must be the world of the launch file (default loop)
    threshold    battery in %, "battery above 40?" (default 40)
    load_time    s, how long loading and unloading take (default 3)
    memory       memory of the sequences recharge, fetch and bring (default true)
    period       s between two ticks (default 0.1, Nav2 ticks every 0.01 s)
    show_tree    true: print the tree with the status of every node when the
                 running leaf changes (default true)
"""
import math
import operator
import signal
from typing import Any, List, Optional

from delivery_bt.worlds import get_world, station_in_map, World
from geometry_msgs.msg import PoseStamped
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
import py_trees
from py_trees.common import Access, ComparisonExpression, Status
import py_trees_ros
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import BatteryState
from std_srvs.srv import Trigger
import tf2_ros

NAMES = {'dock': 'dock', 'pickup': 'pickup', 'dropoff': 'drop off'}


# --- data gathering -----------------------------------------------------------------

class BatteryToBlackboard(py_trees_ros.subscribers.ToBlackboard):
    """
    Write the battery to the blackboard: battery in %, charging true or false.

    The subscriber and the message come from py_trees_ros, this class only
    turns the message into the two numbers the tree needs. RUNNING until the
    first message arrives, then SUCCESS on every tick.
    """

    def __init__(self) -> None:
        super().__init__(name='battery to blackboard', topic_name='battery_state',
                         topic_type=BatteryState, qos_profile=QoSProfile(depth=10),
                         blackboard_variables={'battery_state': None},
                         clearing_policy=py_trees.common.ClearingPolicy.NEVER)
        self.blackboard.register_key(key='battery', access=Access.WRITE)
        self.blackboard.register_key(key='charging', access=Access.WRITE)

    def update(self) -> Status:
        status = super().update()   # copies the latest message to battery_state
        if status == Status.SUCCESS:
            msg = self.blackboard.battery_state
            self.blackboard.battery = 100.0 * msg.percentage   # message: 0 to 1
            self.blackboard.charging = (
                msg.power_supply_status == BatteryState.POWER_SUPPLY_STATUS_CHARGING)
            self.feedback_message = f'{self.blackboard.battery:.0f} %'
        return status


# --- conditions -----------------------------------------------------------------------

class BatteryOk(py_trees.behaviour.Behaviour):
    """Condition "battery above 40?": SUCCESS or FAILURE, never RUNNING."""

    def __init__(self, threshold: float) -> None:
        super().__init__(name=f'battery above {threshold:.0f}?')
        self.threshold = threshold
        self.bb = self.attach_blackboard_client(name=self.name)
        self.bb.register_key(key='battery', access=Access.READ)
        self.bb.register_key(key='charging', access=Access.READ)

    def update(self) -> Status:
        if self.bb.charging:   # on the dock: stay until full
            self.feedback_message = f'charging, {self.bb.battery:.0f} %'
            return Status.FAILURE
        self.feedback_message = f'{self.bb.battery:.0f} %'
        return Status.SUCCESS if self.bb.battery > self.threshold else Status.FAILURE


def is_true(name: str, variable: str) -> py_trees.behaviour.Behaviour:
    """Check that a blackboard variable is True, like cond(lambda w: w["loaded"])."""
    return py_trees.behaviours.CheckBlackboardVariableValue(
        name=name, check=ComparisonExpression(variable=variable, value=True,
                                              operator=operator.eq))


# --- actions --------------------------------------------------------------------------

def go_to(world: World, station: str) -> py_trees.behaviour.Behaviour:
    """
    Drive to a station with Nav2, an action like goto(x) of slide 16.

    py_trees_ros sends the goal to navigate_to_pose when the leaf starts,
    answers RUNNING while Nav2 drives, SUCCESS or FAILURE with Nav2's result,
    and cancels the goal when a branch to the left interrupts it (halt).
    """
    x, y, yaw = station_in_map(world, station)
    goal = NavigateToPose.Goal()
    goal.pose = PoseStamped()
    goal.pose.header.frame_id = 'map'
    goal.pose.pose.position.x = x
    goal.pose.pose.position.y = y
    goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
    goal.pose.pose.orientation.w = math.cos(yaw / 2.0)
    return py_trees_ros.action_clients.FromConstant(
        name=f'go to {NAMES[station]}', action_type=NavigateToPose,
        action_name='navigate_to_pose', action_goal=goal,
        generate_feedback_message=lambda msg: f'{msg.feedback.distance_remaining:.1f} m to go')


class Charge(py_trees.behaviour.Behaviour):
    """
    Action: plug in and charge. RUNNING until the battery is full.

    Calls the service /dock/charge of the simulator once, when it starts.
    FAILURE if the robot does not stand on the dock.
    """

    def __init__(self) -> None:
        super().__init__(name='charge')
        self.bb = self.attach_blackboard_client(name=self.name)
        self.bb.register_key(key='battery', access=Access.READ)
        self.client: Any = None
        self.future: Any = None

    def setup(self, **kwargs: Any) -> None:
        node: Node = kwargs['node']   # py_trees_ros passes the node of the tree
        self.client = node.create_client(Trigger, '/dock/charge')
        while not self.client.wait_for_service(timeout_sec=3.0):
            node.get_logger().warning('waiting for service /dock/charge ...')

    def initialise(self) -> None:
        self.future = self.client.call_async(Trigger.Request())

    def update(self) -> Status:
        if not self.future.done():
            return Status.RUNNING
        response = self.future.result()
        if response is None or not response.success:
            self.feedback_message = response.message if response else 'no answer'
            return Status.FAILURE
        self.feedback_message = f'{self.bb.battery:.0f} %'
        return Status.SUCCESS if self.bb.battery >= 100.0 else Status.RUNNING


def wait(name: str, seconds: float) -> py_trees.behaviour.Behaviour:
    """Wait some seconds, an action: RUNNING, then SUCCESS. Loading takes a moment."""
    return py_trees.timers.Timer(name=name, duration=seconds)


def do(name: str, variable: str, value: Any) -> py_trees.behaviour.Behaviour:
    """Change the blackboard, an instant action that answers SUCCESS (do of slide 16)."""
    return py_trees.behaviours.SetBlackboardVariable(
        name=name, variable_name=variable, variable_value=value, overwrite=True)


# --- the tree of slide 17 -------------------------------------------------------------

def create_tree(world: World, threshold: float, load_time: float,
                memory: bool) -> py_trees.behaviour.Behaviour:
    def seq(name: str, children: List[py_trees.behaviour.Behaviour],
            mem: bool = False) -> py_trees.composites.Sequence:
        return py_trees.composites.Sequence(name=name, memory=mem, children=children)

    def fb(name: str,
           children: List[py_trees.behaviour.Behaviour]) -> py_trees.composites.Selector:
        return py_trees.composites.Selector(name=name, memory=False, children=children)

    return seq('delivery', [
        BatteryToBlackboard(),
        fb('battery', [
            BatteryOk(threshold),
            seq('recharge', [go_to(world, 'dock'), Charge()], memory)]),
        fb('pickup', [
            is_true('loaded?', 'loaded'),
            seq('fetch', [go_to(world, 'pickup'), wait('load', load_time),
                          do('loaded = True', 'loaded', True)], memory)]),
        fb('dropoff', [
            is_true('delivered?', 'delivered'),
            seq('bring', [go_to(world, 'dropoff'), wait('unload', load_time),
                          do('delivered = True', 'delivered', True)], memory)]),
        go_to(world, 'dock'),
    ])


# --- running it -----------------------------------------------------------------------

def wait_for_nav2(node: Node) -> None:
    """Block until AMCL and Nav2 are active and the robot has a pose in the map."""
    for name in ('amcl', 'bt_navigator'):
        client = node.create_client(GetState, f'/{name}/get_state')
        node.get_logger().info(f'Waiting for {name} to be active ...')
        while True:
            if client.wait_for_service(timeout_sec=1.0):
                future = client.call_async(GetState.Request())
                rclpy.spin_until_future_complete(node, future, timeout_sec=2.0)
                result = future.result()
                if result is not None and result.current_state.label == 'active':
                    break
            rclpy.spin_once(node, timeout_sec=1.0)
        node.destroy_client(client)
    buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(buffer, node, spin_thread=False)
    node.get_logger().info('Waiting for the robot pose in the map (tf map -> base_footprint) ...')
    while not buffer.can_transform('map', 'base_footprint', rclpy.time.Time()):
        rclpy.spin_once(node, timeout_sec=0.2)
    del listener


def main(args=None) -> None:
    # no signal handler of rclpy: after Ctrl+C the node still runs and can cancel its goal
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('delivery')
    world = get_world(node.declare_parameter('world', 'loop').value)
    threshold: float = node.declare_parameter('threshold', 40.0).value
    load_time: float = node.declare_parameter('load_time', 3.0).value
    memory: bool = node.declare_parameter('memory', True).value
    period: float = node.declare_parameter('period', 0.1).value
    show_tree: bool = node.declare_parameter('show_tree', True).value
    log = node.get_logger()

    # w = dict(loaded=False, delivered=False), the battery comes from the topic
    w = py_trees.blackboard.Client(name='main')
    for key in ('loaded', 'delivered', 'battery'):
        w.register_key(key=key, access=Access.WRITE)
    w.loaded, w.delivered = False, False

    tree = py_trees_ros.trees.BehaviourTree(create_tree(world, threshold, load_time, memory))
    print(py_trees.display.unicode_tree(tree.root), flush=True)
    try:
        tree.setup(node=node)   # connects to navigate_to_pose, /dock/charge, /battery_state
        wait_for_nav2(node)
    except KeyboardInterrupt:   # Ctrl+C before the mission started: nothing to halt
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        node.destroy_node()
        rclpy.try_shutdown()
        return

    # From here on Ctrl+C only sets a flag, the loop below halts the tree.
    # ros2 run forwards Ctrl+C to this process, so it often arrives twice.
    stopped = False

    def on_sigint(signum: int, frame: Any) -> None:
        nonlocal stopped
        stopped = True

    signal.signal(signal.SIGINT, on_sigint)

    done = False
    last_tip: Optional[str] = None

    def after_tick(tree: py_trees_ros.trees.BehaviourTree) -> None:
        """Report when the running leaf changes, and stop when the root has finished."""
        nonlocal done, last_tip
        tip = tree.root.tip()
        name = tip.name if tip is not None else ''
        status = tree.root.status
        if name != last_tip or status != Status.RUNNING:
            last_tip = name
            battery = f'{w.battery:3.0f} %' if w.exists('battery') else '  ? %'
            message = f' ({tip.feedback_message})' if tip and tip.feedback_message else ''
            if show_tree:
                print(py_trees.display.unicode_tree(tree.root, show_status=True), flush=True)
            log.info(f'battery {battery}   {status.value:8} {name}{message}')
        if status != Status.RUNNING:
            done = True

    log.info(f'Mission starts in world "{world.name}".')
    tree.add_post_tick_handler(after_tick)
    tree.tick_tock(period_ms=1000.0 * period)
    while rclpy.ok() and not done and not stopped:   # while status == R
        rclpy.spin_once(node, timeout_sec=0.1)
    if done:
        log.info(f'Mission {tree.root.status.value}, battery {w.battery:.0f} %, '
                 f'loaded {w.loaded}, delivered {w.delivered}.')
    else:
        log.info('Stopped. Halting the tree, a running goal is cancelled.')
        tree.root.stop(Status.INVALID)   # halt every running node, like a branch to the left
        for _ in range(5):               # let the cancel request go out
            rclpy.spin_once(node, timeout_sec=0.1)
    tree.shutdown()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
