import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import time
import threading
from math import radians, atan2
from crane_plus_commander.kbhit import KBHit
from pymoveit2 import MoveIt2, GripperInterface
from tf_transformations import euler_from_quaternion, quaternion_from_euler

GRIPPER_MIN = -radians(40.62) + 0.001
GRIPPER_MAX = radians(38.27) - 0.001

def to_gripper_ratio(gripper):
    ratio = (gripper - GRIPPER_MIN) / (GRIPPER_MAX - GRIPPER_MIN)
    return ratio

def from_gripper_ratio(ratio):
    gripper = GRIPPER_MIN + ratio * (GRIPPER_MAX - GRIPPER_MIN)
    return gripper


# Node that computes IK with MoveIt for CRANE+ V2 and sends joint commands
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

    def forward_kinematics(self, joint):
        pose_stamped = self.moveit2.compute_fk(joint)
        p = pose_stamped.pose.position
        [x, y, z] = [p.x, p.y, p.z]
        q = pose_stamped.pose.orientation
        [_, pitch,_] = euler_from_quaternion([q.x, q.y, q.z, q.w])
        return [x, y, z, pitch]

    def inverse_kinematics(self, endtip, elbow_up):
        [x, y, z, pitch] = endtip
        position = [x, y, z]
        yaw = atan2(y, x)
        quat_xyzw = quaternion_from_euler(0.0, pitch, yaw)
        if elbow_up:
            start_joint_state = [0.0, 0.0, -1.57, 0.0]
        else:
            start_joint_state = [0.0, 0.0, 1.57, 0.0]
        joint_state = self.moveit2.compute_ik(
            position, quat_xyzw, start_joint_state=start_joint_state)
        if joint_state is None:
            return None
        d = {}
        for i, name in enumerate(joint_state.name):
            d[name] = joint_state.position[i]
        joint = [d[x] for x in self.joint_names]
        return joint


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
    gripper = 0
    commander.set_max_velocity(0.2)
    commander.move_joint(joint)
    commander.move_gripper(gripper)

    # IK solution type
    elbow_up = True

    # Instance of key reader class
    kb = KBHit()

    print('Press 1/2 3/4 5/6/7/8 9/0 to move joints')
    print('Press a/z s/x d/c f/v g/b to move endtip')
    print('Press e to toggle IK solution')
    print('Press Space for stand pose')
    print('Press Esc to quit')

    # Catch KeyboardInterrupt to avoid Ctrl+C errors
    try:
        while True:
            time.sleep(0.01)
            # Is any key pressed?
            if kb.kbhit():
                c = kb.getch()

                # Forward kinematics
                [x, y, z, pitch] = commander.forward_kinematics(joint)
                ratio = to_gripper_ratio(gripper)

                # Keep previous values
                joint_prev = joint.copy()
                gripper_prev = gripper
                elbow_up_prev = elbow_up

                commander.set_max_velocity(1.0)

                # Branch by pressed key
                if c == '1':
                    joint[0] -= 0.1
                elif c == '2':
                    joint[0] += 0.1
                elif c == '3':
                    joint[1] -= 0.1
                elif c == '4':
                    joint[1] += 0.1
                elif c == '5':
                    joint[2] -= 0.1
                elif c == '6':
                    joint[2] += 0.1
                elif c == '7':
                    joint[3] -= 0.1
                elif c == '8':
                    joint[3] += 0.1
                elif c == '9':
                    gripper -= 0.1
                elif c == '0':
                    gripper += 0.1
                elif c == 'a':
                    x += 0.01
                elif c == 'z':
                    x -= 0.01
                elif c == 's':
                    y += 0.01
                elif c == 'x':
                    y -= 0.01
                elif c == 'd':
                    z += 0.01
                elif c == 'c':
                    z -= 0.01
                elif c == 'f':
                    pitch += 0.1
                elif c == 'v':
                    pitch -= 0.1
                elif c == 'g':
                    ratio += 0.1
                elif c == 'b':
                    ratio -= 0.1
                elif c == 'e':
                    elbow_up = not elbow_up
                    print(f'elbow_up: {elbow_up}')
                    commander.set_max_velocity(0.2)
                elif c == ' ':  # Space key
                    joint = [0.0, 0.0, 0.0, 0.0]
                    gripper = 0
                    commander.set_max_velocity(0.2)
                elif ord(c) == 27:  # Esc key
                    break

                # Inverse kinematics
                if c in 'azsxdcfve':
                    joint = commander.inverse_kinematics([x, y, z, pitch], elbow_up)
                    if joint is None:
                        print('No IK solution')
                        joint = joint_prev.copy()
                        elbow_up = elbow_up_prev
                elif c in 'gb':
                    gripper = from_gripper_ratio(ratio)

                if not (GRIPPER_MIN <= gripper <= GRIPPER_MAX):
                    print('Gripper command out of range')
                    gripper = gripper_prev

                # Send command if there is any change
                if joint != joint_prev:
                    print((f'joint: [{joint[0]:.2f}, {joint[1]:.2f}, '
                           f'{joint[2]:.2f}, {joint[3]:.2f}]'))
                    success = commander.move_joint(joint)
                    if not success:
                        print('move_joint() failed')
                        joint = joint_prev.copy()
                if gripper != gripper_prev:
                    print(f'gripper: {gripper:.2f}')
                    success = commander.move_gripper(gripper)
                    if not success:
                        print('move_gripper() failed')
                        gripper = gripper_prev
    except KeyboardInterrupt:
        thread.join()
    else:
        print('Exit')
        # Move slowly to the end pose
        joint = [0.0, 0.0, 0.0, 0.0]
        gripper = 0
        commander.set_max_velocity(0.2)
        commander.move_joint(joint)
        commander.move_gripper(gripper)

    rclpy.try_shutdown()
