import threading
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from airobot_interfaces.action import StringCommand
import threading
from pymoveit2 import MoveIt2, GripperInterface, MoveIt2State
from math import radians

GRIPPER_MIN = -radians(40.62) + 0.001
GRIPPER_MAX = radians(38.27) - 0.001

def to_gripper_ratio(gripper):
    ratio = (gripper - GRIPPER_MIN) / (GRIPPER_MAX - GRIPPER_MIN)
    return ratio

def from_gripper_ratio(ratio):
    gripper = GRIPPER_MIN + ratio * (GRIPPER_MAX - GRIPPER_MIN)
    return gripper


# Node that accepts action requests and sends commands to MoveIt for CRANE+ V2
class CommanderMoveit(Node):

    def __init__(self):
        super().__init__('commander_moveit')
        self.joint_names = [
            'crane_plus_joint1',
            'crane_plus_joint2',
            'crane_plus_joint3',
            'crane_plus_joint4']
        callback_group = ReentrantCallbackGroup()
        self.moveit2 = MoveIt2(
            node=self,
            joint_names=self.joint_names,
            base_link_name='crane_plus_base',
            end_effector_name='crane_plus_link_endtip',
            group_name='arm',
            callback_group=callback_group,
        )
        self.moveit2.max_velocity = 1.0
        self.moveit2.max_acceleration = 1.0

        gripper_joint_names = ['crane_plus_joint_hand']
        self.gripper_interface = GripperInterface(
            node=self,
            gripper_joint_names=gripper_joint_names,
            open_gripper_joint_positions=[GRIPPER_MIN],
            closed_gripper_joint_positions=[GRIPPER_MAX],
            gripper_group_name='gripper',
            callback_group=callback_group,
        )
        self.gripper_interface.max_velocity = 1.0
        self.gripper_interface.max_acceleration = 1.0

        # Dictionary holding string-pose pairs
        self.poses = {}
        self.poses['zeros'] = [0, 0, 0, 0]
        self.poses['ones'] = [1, 1, 1, 1]
        self.poses['home'] = [0.0, -1.16, -2.01, -0.73]
        self.poses['carry'] = [-0.00, -1.37, -2.52, 1.17]

        # Action server
        self.action_server = ActionServer(
            self,
            StringCommand,
            'manipulation/command',
            execute_callback=self.execute_callback,
            cancel_callback=self.cancel_callback,
            handle_accepted_callback=self.handle_accepted_callback,
            callback_group=callback_group,
        )
        self.goal_handle = None  # Variable storing the active goal
        self.goal_lock = threading.Lock()     # Lock to avoid double execution
        self.execute_lock = threading.Lock()  # Lock to avoid double execution

    def handle_accepted_callback(self, goal_handle):
        with self.goal_lock:  # Avoid double execution in this block
            if self.goal_handle is not None and self.goal_handle.is_active:
                self.get_logger().info('Abort previous goal')
                self.goal_handle.abort()
                self.cancel_joint_and_gripper()
            self.goal_handle = goal_handle   # Update goal info
        goal_handle.execute()                # Execute goal

    def execute_callback(self, goal_handle):
        with self.execute_lock:  # Avoid double execution in this block
            self.get_logger().info(f'command: {goal_handle.request.command}')
            result = StringCommand.Result()
            words = goal_handle.request.command.split()
            if words[0] == 'set_pose':
                self.set_pose(words, result)
            elif words[0] == 'set_gripper':
                self.set_gripper(words, result)
            else:
                result.answer = f'NG {words[0]} not supported'
            self.get_logger().info(f'answer: {result.answer}')
            if goal_handle.is_active:
                if result.answer.startswith('OK'):
                    goal_handle.succeed()
                else:
                    goal_handle.abort()
            return result

    def set_pose(self, words, result):
        if len(words) < 2:
            result.answer = f'NG {words[0]} argument required'
            return
        if not words[1] in self.poses:
            result.answer = f'NG {words[1]} not found'
            return
        self.set_max_velocity(0.5)
        success = self.move_joint(self.poses[words[1]])
        if success:
            result.answer = 'OK'
        else:
            result.answer = f'NG {words[0]} move_joint() failed'

    def set_gripper(self, words, result):
        if len(words) < 2:
            result.answer = f'NG {words[0]} argument required'
            return
        try:
            gripper_ratio = float(words[1])
        except ValueError:
            result.answer = f'NG {words[1]} unsuitable'
            return
        if gripper_ratio < 0.0 or 1.0 < gripper_ratio:
            result.answer = 'NG out of range'
            return
        gripper = from_gripper_ratio(gripper_ratio)
        self.set_max_velocity(0.5)
        success = self.move_gripper(gripper)
        if success:
            result.answer = 'OK'
        else:
            result.answer = f'NG {words[0]} move_gripper() failed'

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Cancel received')
        self.cancel_joint_and_gripper()
        return CancelResponse.ACCEPT

    def cancel_joint_and_gripper(self):
        if self.moveit2.query_state() == MoveIt2State.EXECUTING:
            self.moveit2.cancel_execution()
        if self.gripper_interface.query_state() == MoveIt2State.EXECUTING:
            self.gripper_interface.cancel_execution()

    def move_joint(self, q):
        joint_positions = [
            float(q[0]), float(q[1]), float(q[2]), float(q[3])]
        self.moveit2.move_to_configuration(joint_positions)
        return self.moveit2.wait_until_executed()

    def move_gripper(self, q):
        position = float(q)
        self.gripper_interface.move_to_position(position)
        return self.gripper_interface.wait_until_executed()

    def set_max_velocity(self, v):
        self.moveit2.max_velocity = float(v)


def main():
    print('Start')

    # Initialize ROS client
    rclpy.init()

    # Instance of node class
    commander = CommanderMoveit()

    # Run rclpy.spin() in another thread
    executor = MultiThreadedExecutor()
    thread = threading.Thread(target=rclpy.spin, args=(commander,executor,))
    threading.excepthook = lambda x: ()
    thread.start()

    # Move slowly to the initial pose
    commander.set_max_velocity(0.2)
    commander.move_joint(commander.poses['home'])
    commander.move_gripper(GRIPPER_MAX)
    print('Action server ready')

    # Catch KeyboardInterrupt to avoid Ctrl+C errors
    try:
        input('Press Enter to exit\n')
    except KeyboardInterrupt:
        thread.join()
    else:
        print('Stop action server')
        # Move slowly to the end pose
        commander.set_max_velocity(0.2)
        commander.move_joint(commander.poses['zeros'])
        commander.move_gripper(GRIPPER_MIN)

    rclpy.try_shutdown()
    print('Exit')
