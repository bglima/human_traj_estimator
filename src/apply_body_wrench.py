#!/usr/bin/python3

import rospy
import sys
from select import select

from gazebo_msgs.srv import ApplyBodyWrench
from geometry_msgs.msg import Wrench, Point

from dynamic_reconfigure.server import Server
from gazebo_wrench_teleop_keyboard.cfg import ApplyBodyWrenchConfigConfig

if sys.platform == 'win32':
    import msvcrt
else:
    import termios
    import tty


msg = """
Generates wrenches through the keyboard
---------------------------
To apply an instantaneous (positive or negative) wrenches 
press one of the following key:

a : applies a positive wrenches along the x-axis
d : applies a positive wrenches along the y-axis
g : applies a positive wrenches along the z-axis

s : applies a negative wrenches along the x-axis
e : applies a negative wrenches along the y-axis
h : applies a negative wrenches along the z-axis

To apply continuous wrenches, keeps pressing key
---------------------------

CTRL-C to quit...
"""

def getKey(settings, timeout):
    if sys.platform == 'win32':
        # getwch() returns a string on Windows
        key = msvcrt.getwch()
    else:
        tty.setraw(sys.stdin.fileno())
        # sys.stdin.read() returns a string on Linux
        rlist, _, _ = select([sys.stdin], [], [], timeout)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def saveTerminalSettings():
    if sys.platform == 'win32':
        return None
    return termios.tcgetattr(sys.stdin)

def restoreTerminalSettings(old_settings):
    if sys.platform == 'win32':
        return
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)


def apply_body_wrench_client(body_name, reference_frame, reference_point, wrench, start_time, duration):
    rospy.wait_for_service('/gazebo/apply_body_wrench')
    try:
        apply_body_wrench = rospy.ServiceProxy('/gazebo/apply_body_wrench', ApplyBodyWrench)
        apply_body_wrench(body_name, reference_frame, reference_point, wrench, start_time, duration)
    except rospy.ServiceException as e:
        print ("Service call failed: %s", e)

def dynamic_reconfigure_callback(config, level):
    global target_force_x
    global target_force_y
    global target_force_z
    target_force_x = config.force_x
    target_force_y = config.force_y
    target_force_z = config.force_z
    return config

if __name__=="__main__":

    settings = saveTerminalSettings()

    rospy.init_node('wrench_teleop_keyboard')
    srv = Server(ApplyBodyWrenchConfigConfig, dynamic_reconfigure_callback)

    pub = rospy.Publisher('forces', Wrench, queue_size=10)

    wrench = Wrench()
    wrench.force.x = 0
    wrench.force.y = 0
    wrench.force.z = 0
    wrench.torque.x = 0
    wrench.torque.y = 0
    wrench.torque.z = 0

    body_name = rospy.get_param("apply_body_wrench/body_name", "panda_link7")
    reference_frame = rospy.get_param("/apply_body_wrench/reference_frame", "world")
    reference_point_x = rospy.get_param("/apply_body_wrench/reference_point/x")
    reference_point_y = rospy.get_param("/apply_body_wrench/reference_point/y")
    reference_point_z = rospy.get_param("/apply_body_wrench/reference_point/z")
    start_time_secs = rospy.get_param("/apply_body_wrench/start_time/secs")
    start_time_nsecs = rospy.get_param("/apply_body_wrench/start_time/nsecs")
    duration_secs = rospy.get_param("/apply_body_wrench/duration/secs")
    duration_nsecs = rospy.get_param("/apply_body_wrench/duration/nsecs")
    key_timeout = rospy.get_param("/apply_body_wrench/key_timeout", 1)

    reference_point = Point(reference_point_x, reference_point_y, reference_point_z)
    start_time = rospy.Time(start_time_secs, start_time_nsecs)
    duration = rospy.Duration(duration_secs, duration_nsecs)

    default_keys = {
            'a': 20,
            'd': 20,
            'g': 20,
            's': -20,
            'e': -20,
            'h': -20
        }

    try:
        rospy.sleep(3)
        print(msg)

        while(1):

            default_keys["a"] = target_force_x
            default_keys["d"] = target_force_y
            default_keys["g"] = target_force_z
            default_keys["s"] = - target_force_x
            default_keys["e"] = - target_force_y
            default_keys["h"] = - target_force_z

            # Get the pressed key
            key = getKey(settings, key_timeout)

            # If the key corresponds to a key in default_keys
            if key in default_keys.keys():
                if key == "a" or key == "s":
                    wrench.force.x = default_keys[key]
                elif key == "d" or key == "e":
                    wrench.force.y = default_keys[key]
                else:
                    wrench.force.z = default_keys[key]
            elif key == '\x03':  # if ctrl-C (^C) was pressed, terminate the program
                break
            else:
                wrench.force.x = 0
                wrench.force.y = 0
                wrench.force.z = 0
            apply_body_wrench_client(body_name, reference_frame,reference_point, wrench, start_time, duration)
            pub.publish(wrench)

    except Exception as e:
        print(e)

    finally:
        restoreTerminalSettings(settings)
