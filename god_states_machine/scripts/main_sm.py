#!/usr/bin/env python

import rospy
import smach
import smach_ros
#import god_state_machine as gsm
import states_machine as gsm

def main():
    rospy.init_node('smach_example_state_machine')

    start_state = smach.StateMachine(outcomes=['god_is_gone'])

    with start_state:
        smach.StateMachine.add('START', gsm.Start(),
                               transitions={'started': 'SPEECHRECOGITION'})
<<<<<<< HEAD


        # smach.StateMachine.add('READYPICK', gsm.ReadyPick(),
        #                        transitions={'ready_pick': 'PICKBAG'})
        # smach.StateMachine.add('PICKBAG', gsm.Pickbag(),
        #                        transitions={'pick_done': 'FOLLOWTOP'})

        smach.StateMachine.add('READYPICK', gsm.ReadyPick(),
                               transitions={'pick_done': 'FOLLOWTOP'})


        smach.StateMachine.add('SPEECHRECOGITION', gsm.SpeechRecognition(),
                               transitions={'waitcommand': 'SPEECHRECOGITION',
                                            'localized': 'CLEARCOSTMAP',
                                            'relocalizing': 'LOCALIZATION',
                                            'keepfollow': 'FOLLOWME',
                                            'bring_bag': 'GIVEBAG',
                                            'ready': 'READYPICK'})

        smach.StateMachine.add('FOLLOWTOP', gsm.FollowTop(),
                               transitions={'followme': 'LOCALIZATION'})
        smach.StateMachine.add('LOCALIZATION', gsm.Localization(),
                               transitions={'localizing': 'SPEECHRECOGITION'})
        smach.StateMachine.add('CLEARCOSTMAP', gsm.Clearcostmap(),
                               transitions={'cleared': 'LEARN_OPERATOR'})
        smach.StateMachine.add('LEARN_OPERATOR', gsm.Learn_operator(),
                               transitions={'learning_done': 'SHOW_LEARN'})
        smach.StateMachine.add('SHOW_LEARN', gsm.Show_learn(),
                               transitions={'learnshown': 'STARTFOLLOW'})
        smach.StateMachine.add('STARTFOLLOW', gsm.StartFollow(),
                               transitions={'ready_follow': 'FOLLOWME'})
                               #remapping={'next_desired_pose', 'tempt_desired_pose'})
        smach.StateMachine.add('FOLLOWME', gsm.Followme(),
                               transitions={'following': 'IFLOSE',
                                            'wrong': 'IFLOSE',
                                            'done': 'GIVEBAG'})
        smach.StateMachine.add('IFLOSE', gsm.Iflose(),
                               transitions={'notlose': 'FOLLOWME',
                                            'lose': 'SHOW_LEARN'})

        smach.StateMachine.add('GIVEBAG', gsm.Givebag(),
                               transitions={'baggiven': 'END'})
        smach.StateMachine.add('END', gsm.End(),
                               transitions={'end': 'god_is_gone'})

    sis = smach_ros.IntrospectionServer('my_smach_introspection_server', start_state, '/SM_god')
    sis.start()

    outcome = start_state.execute()
    rospy.spin()

    sis.stop()

if __name__ == '__main__':
    main()
    #
    # # Open the container
    #     smach.StateMachine.add('FOLLOWTOP', gsm.FollowTop(),
    #                            transitions={'followme': 'FOLLOW_SUB'})
    #
    #     # Create the sub SMACH state machine
    #     follow_sub = smach.StateMachine(outcomes=['outcome4'])
    #
    #     # Open the container
    #     with follow_sub:
    #         # Add states to the container



        # smach.StateMachine.add('FOLLOW_SUB', follow_sub,
        #                        transitions={'outcome4': 'end'})

=======
        smach.StateMachine.add('READYPICK', gsm.ReadyPick(),
                               transitions={'ready_pick': 'SPEECHRECOGITION'})
        smach.StateMachine.add('SPEECHRECOGITION', gsm.SpeechRecognition(),
                               transitions={'waitcommand': 'SPEECHRECOGITION',
                                            'notgetbag': 'READYPICK',
                                            'getbag': 'FOLLOWTOP',
                                            'localized': 'CLEARCOSTMAP',
                                            'relocalizing': 'LOCALIZATION',
                                            'keepfollow': 'FOLLOWME',
                                            'bring_bag': 'GIVEBAG'})
        smach.StateMachine.add('FOLLOWTOP', gsm.FollowTop(),
                               transitions={'followme': 'LOCALIZATION'})
        smach.StateMachine.add('LOCALIZATION', gsm.Localization(),
                               transitions={'localizing': 'SPEECHRECOGITION'})
        smach.StateMachine.add('CLEARCOSTMAP', gsm.Clearcostmap(),
                               transitions={'cleared': 'LEARN_OPERATOR'})
        smach.StateMachine.add('LEARN_OPERATOR', gsm.Learn_operator(),
                               transitions={'learning_done': 'STARTFOLLOW'})
        smach.StateMachine.add('STARTFOLLOW', gsm.StartFollow(),
                               transitions={'ready_follow': 'FOLLOWME'},
                               remapping={'next_desired_pose', 'tempt_desired_pose'})
        smach.StateMachine.add('FOLLOWME', gsm.Followme(),
                               transitions={'following': 'SPEECHRECOGITION',
                                            'wrong': 'END',
                                            'done': 'END'},
                               remapping={'desired_pose': 'tempt_desired_pose',
                                          'next_desired_pose': 'tempt_desired_pose'})
        smach.StateMachine.add('GIVEBAG', gsm.Givebag(),
                               transitions={'baggiven': 'END'})
        smach.StateMachine.add('END', gsm.End(),
                               transitions={'end': 'god_is_gone'})

    sis = smach_ros.IntrospectionServer('my_smach_introspection_server', start_state, '/SM_ROOT')
    sis.start()

    outcome = start_state.execute()
    rospy.spin()

    sis.stop()

if __name__ == '__main__':
    main()
    #
    # # Open the container
    #     smach.StateMachine.add('FOLLOWTOP', gsm.FollowTop(),
    #                            transitions={'followme': 'FOLLOW_SUB'})
    #
    #     # Create the sub SMACH state machine
    #     follow_sub = smach.StateMachine(outcomes=['outcome4'])
    #
    #     # Open the container
    #     with follow_sub:
    #         # Add states to the container



        # smach.StateMachine.add('FOLLOW_SUB', follow_sub,
        #                        transitions={'outcome4': 'end'})

>>>>>>> cc9392bc67e44b10c1c80bc4ba0a97209018581e
    # Execute SMACH plan

