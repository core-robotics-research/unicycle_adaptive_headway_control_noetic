#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, PoseStamped
from tf.transformations import euler_from_quaternion

from unicycle_motion_control import adaptive_headway_ctrl


class AdaptiveHeadwayControl:
    def __init__(self):

        # Initialize state variables
        self.goal_position = [0.0, 0.0]
        self.position = [0.0, 0.0]
        self.orientation = 0.0
        self.rate = 10
        self.epsilongain = 0.5
        self.frame_id = "world"
        
    def pose_callback(self, msg):
        # Get initial pose information from message
        self.position = [msg.pose.position.x, msg.pose.position.y]
        quat = (msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w)
        self.orientation = euler_from_quaternion(quat)[2]
        self.frame_id = msg.header.frame_id

    def goal_pose_callback(self, msg):
        # This function will be called when a new PoseStamped message is received
        self.goal_position = [msg.pose.position.x, msg.pose.position.y]


    def start(self):

        # Create and register the ROS node with the master
        rospy.init_node("adaptive_headway_control", anonymous=True)

        # Set up subscriber to get initial pose information
        rospy.Subscriber("pose", PoseStamped, self.pose_callback)

        self.goal_position = self.position
        # Create a subscriber to the "goal" topic
        rospy.Subscriber("goal", PoseStamped, self.goal_pose_callback, queue_size=1)

        # Set up publisher to publish PoseStamped messages for new pose information
        self.cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)

        # Set up control loop to run at 10 Hz
        self.rate = rospy.get_param("~rate", default=10.0) 

        # Set up epsilon control gain
        self.epsilongain = rospy.get_param("~epsilon_gain", default=0.5) 

        # Set up reference control gain
        self.refgain = rospy.get_param("~reference_gain", default=1.0)

        # Set up tolerance for goal convergence
        self.tol = rospy.get_param("~convergence_tolerance", default=1e-3)

        # Set up direction of motion
        self.motion_direction = rospy.get_param("~motion_direction", default="bidirectional")

        # Publish command velocities as Twist message
        cmd_vel_msg = Twist()

        rate = rospy.Rate(self.rate)
        while not rospy.is_shutdown():

            # Integrate unicycle dynamics to get new pose
            linvel, angvel = adaptive_headway_ctrl(self.position, self.orientation, self.goal_position, self.epsilongain, self.refgain, self.tol, self.motion_direction)

            cmd_vel_msg.linear.x = linvel
            cmd_vel_msg.angular.z = angvel
            self.cmd_vel_pub.publish(cmd_vel_msg)

            rate.sleep()

if __name__ == '__main__':
    try:
        ahc_controller = AdaptiveHeadwayControl().start()
    except rospy.ROSInterruptException:
        pass