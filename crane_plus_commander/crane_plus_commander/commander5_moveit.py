import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.duration import Duration
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf_transformations import euler_from_quaternion, quaternion_from_euler
import time
import threading
from math import sqrt, radians, atan2
from crane_plus_commander.kbhit import KBHit
from pymoveit2 import MoveIt2, GripperInterface

GRIPPER_MIN = -radians(40.62) + 0.001
GRIPPER_MAX = radians(38.27) - 0.001


# Node that uses MoveIt for CRANE+ V2 to compute IK and position the endtip at a point given by a tf frame
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

        # tf
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

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

    def move_endtip(self, endtip):
        position = [float(endtip[0]), float(endtip[1]), float(endtip[2])]
        yaw = atan2(position[1], position[0])
        pitch = float(endtip[3])
        quat_xyzw = quaternion_from_euler(0.0, pitch, yaw)
        self.moveit2.move_to_pose(
            position=position,
            quat_xyzw=quat_xyzw
        )
        return self.moveit2.wait_until_executed()

    def get_frame_position(self, frame_id):
        try:
            when = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform(
                'crane_plus_base',
                frame_id,
                when,
                timeout=Duration(seconds=1.0))
        except TransformException as ex:
            self.get_logger().info(f'{ex}')
            return None
        t = trans.transform.translation
        r = trans.transform.rotation
        roll, pitch, yaw = euler_from_quaternion([r.x, r.y, r.z, r.w])
        return [t.x, t.y, t.z, roll, pitch, yaw]


# Compute distance between 3D points represented as lists
def dist(a, b):
    return sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2 + (a[2] - b[2])**2)

def main():
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
    joint = [0.0, -1.16, -2.01, -0.73]
    gripper = GRIPPER_MIN
    commander.set_max_velocity(0.2)
    commander.move_joint(joint)
    commander.move_gripper(gripper)

    # IK solution type
    elbow_up = True

    # Instance of key reader class
    kb = KBHit()

    # States
    INIT = 0
    WAIT = 1
    DONE = 2
    state = INIT

    print('Press r to reinitialize')
    print('Press Esc to quit')

    # Catch KeyboardInterrupt to avoid Ctrl+C errors
    try:
        while True:
            time.sleep(0.01)            
            # Is any key pressed?
            if kb.kbhit():
                c = kb.getch()
                if c == 'r':
                    print('Reinitialize')
                    state = INIT
                elif ord(c) == 27:  # Esc key
                    break

            position = commander.get_frame_position('target')
            if position is None:
                print('Target frame not found')
            else:
                xyz_now = position[0:3]
                time_now = time.time()
                if state == INIT:
                    xyz_first = xyz_now
                    time_first = time_now
                    state = WAIT
                elif state == WAIT:
                    if dist(xyz_now, xyz_first) > 0.01:
                        state = INIT
                    elif time_now - time_first > 1.0:
                        state = DONE
                        commander.set_max_velocity(1.0)
                        pitch = 0
                        sucess = commander.move_endtip(xyz_now + [pitch])
                        if sucess:
                            print('move_endtip() succeeded')
                        else:
                            print('move_endtip() failed')
    except KeyboardInterrupt:
        thread.join()
    else:
        print('Exit')
        # Move slowly to the end pose
        joint = [0.0, 0.0, 0.0, 0.0]
        gripper = GRIPPER_MAX
        commander.set_max_velocity(0.2)
        commander.move_joint(joint)
        commander.move_gripper(gripper)

    rclpy.try_shutdown()
