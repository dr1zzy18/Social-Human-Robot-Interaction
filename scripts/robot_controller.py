#!/usr/bin/env python

import sys
import rospy
from cr_week6_test.msg import perceived_info, robot_info
import random
from cr_week6_test.srv import predict_robot_expression

# Create a service proxy for the predict_robot_expression service
proxy_predict_robot_expression = rospy.ServiceProxy('predict_robot_expression_service', predict_robot_expression, persistent=True)
# Create a publisher for the robot_info topic
pub = rospy.Publisher('robotInfo', robot_info, queue_size=10)

# Define a callback function to handle incoming perceived_info messages
def callback(data):
    global proxy_predict_robot_expression
    global pub
    # Wait for the predict_robot_expression service to become available
    rospy.wait_for_service('predict_robot_expression_service')
    try:
        # Call the predict_robot_expression service with the perceived_info data
        response = proxy_predict_robot_expression(human_action = data.human_action, human_expression = data.human_expression, object_size = data.object_size)
        # Create a robot_info message with the predicted probabilities
        robot_probabilities = robot_info(id = data.id, p_happy = response.p_happy, p_sad = response.p_sad, p_neutral = response.p_neutral)
        # Publish the robot_info message to the robotInfo topic
        pub.publish(robot_probabilities)
        rospy.loginfo(robot_probabilities)
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

# Define a listener function to wait for incoming perceived_info messages
def listener():
    global robot_probabilities
    # Initialize the ROS node
    rospy.init_node('robotController', anonymous=True)
    # Subscribe to the perceivedInfo topic and call the callback function for each message
    rospy.Subscriber('perceivedInfo', perceived_info, callback,queue_size=10)
    rospy.spin()


if __name__ == '__main__':
    listener()
