#!/usr/bin/env python

import rospy

import smach

from smach import StateMachine

def main():
    rospy.init_node('smach_usecase_executive_step_02')


    # Create a SMACH state machine
    sm_root = StateMachine(outcomes=[])

    # Open the container
    with sm_root:
	pass
	
    outcome = sm_root.execute()
    # Signal ROS shutdown (kill threads in background)
    rospy.spin()

if __name__ == '__main__':
    main()
