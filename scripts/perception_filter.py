#!/usr/bin/env python
import rospy
from cr_week6_test.msg import human_info
from cr_week6_test.msg import object_info
from cr_week6_test.msg import perceived_info
import random
import message_filters

# Initialize the publisher for perceived information messages
pub = rospy.Publisher('perceivedInfo', perceived_info, queue_size=10)

# Define the callback function for synchronized messages
def callback(hdata, odata):
    global pub
    # Generate a random filter
    filter = random.randint(1,8)
    # Create an instance of perceived information message
    p_info = perceived_info()

    # Apply filters to create the perceived information message
    if filter != 2 and filter != 4 and filter != 6 and filter != 7:
        p_info.human_action = hdata.human_action
    else:
        p_info.human_action = 0

    if filter != 3 and filter != 5 and filter != 6 and filter != 7:
        p_info.human_expression = hdata.human_expression
    else:
        p_info.human_expression = 0

    if filter != 1 and filter != 4 and filter != 5 and filter != 7:
        p_info.object_size = odata.object_size
    else:
        p_info.object_size = 0

    p_info.id = hdata.id
    # Publish the perceived information message
    pub.publish(p_info)

# Define the listener function
def listener():
    # Initialize the ROS node
    rospy.init_node('perceptionFilter', anonymous=True)
    # Subscribe to the 'humanInfo' topic
    subsciber1 = message_filters.Subscriber('humanInfo', human_info)
    # Subscribe to the 'objectInfo' topic
    subscriber2 = message_filters.Subscriber('objectInfo', object_info)
    # Synchronize the messages from both the topics
    timeSynchronizer = message_filters.ApproximateTimeSynchronizer([subsciber1, subscriber2], queue_size = 10, slop = 1, allow_headerless=True)
    timeSynchronizer.registerCallback(callback)
    rospy.spin()

if __name__ == '__main__':
    # Call the listener function
    listener()
