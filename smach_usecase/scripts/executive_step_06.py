#!/usr/bin/env python

import rospy

import threading

import smach

from smach_ros import ServiceState, SimpleActionState, IntrospectionServer, set_preempt_handler

from smach import StateMachine, Concurrence

import std_srvs.srv
import turtlesim.srv
import turtle_actionlib.msg


def main():
    rospy.init_node('smach_usecase_executive')

    # Construct static goals
    polygon_big = turtle_actionlib.msg.ShapeGoal(edges = 11, radius = 4.0)
    polygon_small = turtle_actionlib.msg.ShapeGoal(edges = 6, radius = 0.5) 

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
                    request = turtlesim.srv.SpawnRequest(0.0,0.0,0.0,'turtle2')),
                {'succeeded':'TELEPORT1'})

        # Teleport turtle 1
        StateMachine.add('TELEPORT1',
                ServiceState('turtle1/teleport_absolute', turtlesim.srv.TeleportAbsolute,
                    request = turtlesim.srv.TeleportAbsoluteRequest(5.0,1.0,0.0)),
                {'succeeded':'TELEPORT2'})

        # Teleport turtle 2
        StateMachine.add('TELEPORT2',
                ServiceState('turtle2/teleport_absolute', turtlesim.srv.TeleportAbsolute,
                    request = turtlesim.srv.TeleportAbsoluteRequest(9.0,5.0,0.0)),
                {'succeeded':'DRAW_SHAPES'})


	shape_cc = Concurrence(
			outcomes=['succeeded','aborted','preempted'],
			default_outcome='aborted',
			outcome_map ={'succeeded':{'BIG':'succeeded','SMALL':'succeeded'}})
	
	StateMachine.add('DRAW_SHAPES',shape_cc)
	
	with shape_cc:
		Concurrence.add("BIG", 
				SimpleActionState('turtle_shape1', turtle_actionlib.msg.ShapeAction,
		                goal = polygon_big))

		Concurrence.add("SMALL", 
				SimpleActionState('turtle_shape2', turtle_actionlib.msg.ShapeAction,
		                goal = polygon_small))

    # Attach a SMACH introspection server
    sis = IntrospectionServer('smach_usecase_01', sm_root, '/USE_CASE')
    sis.start()
    
    # Set preempt handler
    set_preempt_handler(sm_root)

    # Execute SMACH tree in a separate thread so that we can ctrl-c the script
    smach_thread = threading.Thread(target = sm_root.execute)
    smach_thread.start()

    # Signal ROS shutdown (kill threads in background)
    rospy.spin()

    sis.stop()

if __name__ == '__main__':
    main()
