#!/usr/bin/env python

import rospy
from cr_week6_test.msg import human_info
from cr_week6_test.msg import object_info
import random

# Initialize global ID counter
id_counter = 0

# Define the interaction generator function
def interaction_generator():
    # Declare the global ID counter variable
    global id_counter
    # Initialize the ROS node and publishers
    pub1 = rospy.Publisher('humanInfo', human_info, queue_size=10)
    pub2 = rospy.Publisher('objectInfo', object_info, queue_size=10)
    rospy.init_node('interactionGenerator', anonymous=True)
    # Set the rate at which the loop will execute
    rate = rospy.Rate(0.1)
    # Loop until ROS is shut down
    while not rospy.is_shutdown():
        # Create instances of human_info and object_info messages
        humaninfo = human_info()
        objectinfo = object_info()
        # Assign the current ID to each message and increment the ID counter
        humaninfo.id = id_counter
        objectinfo.id = id_counter
        id_counter += 1
        # Generate random values for the message fields
        objectinfo.object_size = random.randint(1,2)
        humaninfo.human_expression = random.randint(1,3)
        humaninfo.human_action = random.randint(1,3)
        # Publish the messages
        pub1.publish(humaninfo)
        pub2.publish(objectinfo)
        # Sleep to maintain the specified loop rate
        rate.sleep()

if __name__ == '__main__':
    try:
        interaction_generator()
    except rospy.ROSInterruptException:
        pass
