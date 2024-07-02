#!/usr/bin/env python

import rospy
import geometry_msgs.msg
import franka_msgs.msg
import math
import std_msgs.msg
import numpy as np
import tf.transformations as tr

def euler_from_quaternion(x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z # in radians

def updatedcallback(data):
    # rospy.loginfo(rospy.get_caller_id() + ' values obtained from the /human_target topic are: %s', data)

    # converting the quaternion values to the rpy angles
    RPY = euler_from_quaternion(data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w)
    # print("RPY updated values stored in a vector")
    # print(RPY)

    # Publishing the error to a new topic 
    dt = 0.008
    rate = rospy.Rate(1/dt)

    pub1 = rospy.Publisher("/Roll_updated", std_msgs.msg.Float32, queue_size=1)
    pub2 = rospy.Publisher("/Pitch_updated", std_msgs.msg.Float32, queue_size=1)
    pub3 = rospy.Publisher("/Yaw_updated", std_msgs.msg.Float32, queue_size=1)

    pub1.publish(RPY[0])
    pub2.publish(RPY[1])
    pub3.publish(RPY[2])

    rate.sleep()

def measuredcallback(data):
    # rospy.loginfo(rospy.get_caller_id() + ' values obtained from the /franka_state_controller/franka_states topic are: %s', data.O_T_EE)

    rotation_matrix = np.array([[data.O_T_EE[0], data.O_T_EE[4], data.O_T_EE[8]],
                                [data.O_T_EE[1], data.O_T_EE[5], data.O_T_EE[9]],
                                [data.O_T_EE[2], data.O_T_EE[6], data.O_T_EE[10]]])
    
    # print("rotation matrix:")
    # print(rotation_matrix)

    # Converting the rotation matrix into quaternion form
    RPY = tr.euler_from_matrix(rotation_matrix)
    # print("RPY measured values stored in a vector")
    # print(RPY)

    # Publishing the error to a new topic 
    dt = 0.008
    rate = rospy.Rate(1/dt)

    pub1 = rospy.Publisher("/Roll_measured", std_msgs.msg.Float32, queue_size=1)
    pub2 = rospy.Publisher("/Pitch_measured", std_msgs.msg.Float32, queue_size=1)
    pub3 = rospy.Publisher("/Yaw_measured", std_msgs.msg.Float32, queue_size=1)

    pub1.publish(RPY[0])
    pub2.publish(RPY[1])
    pub3.publish(RPY[2])

    rate.sleep()
    
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('receiver', anonymous=True)

    # Subscription to the topics 
    rospy.Subscriber("/human_ref", geometry_msgs.msg.PoseStamped, updatedcallback)
    rospy.Subscriber("/franka_state_controller/franka_states", franka_msgs.msg.FrankaState, measuredcallback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__': 
    try: 
        listener() 
    except rospy.ROSInterruptException: 
        pass
