#!/usr/bin/env python

import rospy
import tf
from geometry_msgs.msg import PoseStamped
from asctec_hl_comm.msg import mav_ctrl

pub = None

def callback(data):
    angles = tf.transformations.euler_from_quaternion([data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z,data.pose.orientation.w])

    msg = mav_ctrl()
    msg.header = data.header
    msg.type = mav_ctrl.position
    msg.x = data.pose.position.x
    msg.y = data.pose.position.y
    msg.z = data.pose.position.z
    msg.yaw = angles[2]
    msg.v_max_xy = -1 #hardcoded, refer to asctec files
    msg.v_max_z = -1

    pub.publish(msg)



#listens to PoseStamped
def listener():
    global pub
    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that ospy will choose a unique
    # name for our 'talker' node so that multiple talkers can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)
    mav_publisher = rospy.get_param("~mav_publisher_name")
    poseStamped_listener = rospy.get_param("~poseStamped_listener_name")

    #Publishes mav.ctrl
    pub = rospy.Publisher(mav_publisher, mav_ctrl, queue_size=10)

    rospy.Subscriber(poseStamped_listener, PoseStamped, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()



if __name__ == '__main__':
    listener()
