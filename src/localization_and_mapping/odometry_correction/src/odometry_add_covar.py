#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry

pub = rospy.Publisher('/odometry_covar', Odometry, queue_size=10) 

def callback(data):
    global pub
    # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    new_data= Odometry()
    # new_data.pose.covariance = data.pose.covariance
    covar = list(data.pose.covariance) 
    data.header.frame_id="/base_footprint"
    covar[0]=1
    covar[7]=1
    covar[14]=1
    covar[21]=10000
    covar[28]=10000
    covar[35]=10000
    data.pose.covariance=tuple(covar)
    pub.publish(data)
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('add_covariance_node', anonymous=True)

    rospy.Subscriber("odometrija", Odometry, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
