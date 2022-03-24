#!/usr/bin/env python

import rospy

import smach

from smach_ros import ServiceState

from smach import StateMachine

import std_srvs.srv
import turtlesim.srv
import turtle_actionlib.msg


def main():
    rospy.init_node('smach_usecase_executive')

    # Create a SMACH state machine
    sm_root = StateMachine(outcomes=['succeeded','aborted','preempted'])

    # Open the container
    with sm_root:
	# Reset turtlesim
	StateMachine.add('RESET',
                ServiceState('reset', std_srvs.srv.Empty),
			    {'succeeded':'SPAWN'})

        # Create a second turtle
        StateMachine.add('SPAWN',
                ServiceState('spawn', turtlesim.srv.Spawn,
                    request = turtlesim.srv.SpawnRequest(5.0,0.0,0.0,'turtle2')))

       

    # Execute SMACH tree in a separate thread so that we can ctrl-c the script
    outcome = sm_root.execute()


    # Signal ROS shutdown (kill threads in background)
    rospy.spin()


if __name__ == '__main__':
    main()
