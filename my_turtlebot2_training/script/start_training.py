#!/usr/bin/env python
from functools import reduce

import gym
import numpy
import time
import qlearn
from gym import wrappers

# ROS packages required
import rospy
import rospkg
from openai_ros.openai_ros_common import StartOpenAI_ROS_Environment

from openai_ros.msg import TrainingInfo
import subprocess


if __name__ == '__main__':
    rospy.init_node('example_turtlebot2_maze_qlearn', anonymous=True, log_level=rospy.WARN)

    # Init OpenAI_ROS ENV
    task_and_robot_environment_name = rospy.get_param('/turtlebot2/task_and_robot_environment_name')
    env = StartOpenAI_ROS_Environment(task_and_robot_environment_name)

    # Create the Gym environment
    rospy.loginfo("Gym environment done")
    rospy.loginfo("Starting Learning")

    # Set the logging system
    rospack = rospkg.RosPack()
    pkg_path = rospack.get_path('turtle2_openai_ros_example')
    outdir = pkg_path + '/training_results'
    env = wrappers.Monitor(env, outdir, force=True)
    rospy.loginfo("Monitor Wrapper started")

    last_time_steps = numpy.ndarray(0)

    # Loads parameters from the ROS param server
    # Parameters are stored in a yaml file inside the config directory
    # They are loaded at runtime by the launch file
    Alpha = rospy.get_param("/turtlebot2/alpha")
    Epsilon = rospy.get_param("/turtlebot2/epsilon")
    Gamma = rospy.get_param("/turtlebot2/gamma")
    epsilon_discount = rospy.get_param("/turtlebot2/epsilon_discount")
    nepisodes = rospy.get_param("/turtlebot2/nepisodes")
    nsteps = rospy.get_param("/turtlebot2/nsteps")

    running_step = rospy.get_param("/turtlebot2/running_step")

    # Initialises the algorithm that we are going to use for learning
    qlearn = qlearn.QLearn(actions=range(env.action_space.n), alpha=Alpha, gamma=Gamma, epsilon=Epsilon)

    initial_epsilon = qlearn.epsilon

    start_time = time.time()
    highest_reward = 0

    training_info_pub = rospy.Publisher('/training_info', TrainingInfo, queue_size=1)

    # Starts the main training loop: the one about the episodes to do
    for x in range(nepisodes):

        if not rospy.is_shutdown():
            rospy.logdebug("############### EMPTY START EPISODE=>" + str(x))

            cumulated_reward = 0
            done = False
            if qlearn.epsilon > 0.05:
                qlearn.epsilon *= epsilon_discount
            else:
                print("STOPPED DISCOUNTING THE EPSILON")

            # Initialize the environment and get first state of the robot
            observation = env.reset()
            state = ''.join(map(str, observation))

            # Show on screen the actual situation of the robot
            # env.render()
            # for each episode, we test the robot for nsteps
            for i in range(nsteps):
                if not rospy.is_shutdown():
                    rospy.logwarn("# Start Step=>" + str(i))
                    # Pick an action based on the current state
                    action, q_value = qlearn.chooseAction(state, return_q=True)

                    # rospy.logwarn("Next action is:", action)
                    # Execute the action in the environment and get feedback
                    observation, reward, done, info = env.step(action)  # the function step() will call _set_action()
                    # choose_action -> set_action -> learn_q

                    training_info = TrainingInfo()
                    training_info.episode = x
                    training_info.step = i
                    training_info.observation = observation
                    training_info.action = action
                    training_info.reward = reward
                    training_info.q_value = qlearn.getQ(state, action)
                    training_info_pub.publish(training_info)

                    # rospy.logwarn(str(observation) + " " + str(reward))
                    cumulated_reward += reward
                    if highest_reward < cumulated_reward:
                        highest_reward = cumulated_reward

                    nextState = ''.join(map(str, observation))

                    # Make the algorithm learn based on the results
                    # rospy.logwarn("# state we were=>" + str(state))
                    # rospy.logwarn("# action that we took=>" + str(action))
                    # rospy.logwarn("# reward that action gave=>" + str(reward))
                    rospy.logwarn("# episode cumulated_reward=>" + str(cumulated_reward))
                    # print("\n")
                    # rospy.logwarn("# State in which we will start next step=>" + str(nextState))
                    qlearn.learn(state, action, reward, nextState)

                    print("THE CHOSEN ACTION: " + str(action) + ",\tGIVEN REWARD: " + str(reward) + ",\tCALCULATED Q-VALUE: " + str(qlearn.getQ(state, action)))
                    print("Epsilon: " + str(qlearn.epsilon) + ",\tState: " + str(state))
                    print("Q(0): " + str(qlearn.getQ(state, 0)) + ",\tQ(1): " + str(qlearn.getQ(state, 1)) + ",\tQ(2): " + str(qlearn.getQ(state, 2)))
                    print("---------------------------------------------------------------------------\n")

                    if not done:
                        # rospy.logwarn("NOT DONE")
                        state = nextState
                    else:
                        rospy.logwarn("DONE")
                        last_time_steps = numpy.append(last_time_steps, [int(i + 1)])
                        break
                    # rospy.logwarn("############### END Step=>" + str(i))

                    # raw_input("Next Step...PRESS KEY")
                    # rospy.sleep(running_step)
                else:
                    p = subprocess.Popen("killall -9 gazebo & killall -9 gzserver  & killall -9 gzclient", shell=True)
                    break

            m, s = divmod(int(time.time() - start_time), 60)
            h, m = divmod(m, 60)
            rospy.logerr(("EP: " + str(x + 1) + " - [alpha: " + str(round(qlearn.alpha, 2)) + " - gamma: " + str(
                round(qlearn.gamma, 2)) + " - epsilon: " + str(round(qlearn.epsilon, 2)) + "] - Reward: " + str(
                cumulated_reward) + "     Time: %d:%02d:%02d" % (h, m, s)))

        else:
            p = subprocess.Popen("killall -9 gazebo & killall -9 gzserver  & killall -9 gzclient", shell=True)
            break

        # rospy.logwarn(("\n|" + str(nepisodes) + "|" + str(qlearn.alpha) + "|" + str(qlearn.gamma) + "|" +
        #                str(initial_epsilon) + "*" + str(epsilon_discount) + "|" + str(highest_reward) + "| PICTURE |\n"))

        l = last_time_steps.tolist()
        l.sort()

    print("****************** Q TABLE ****************************************************************")
    print(qlearn.q)
    print("*******************************************************************************************")

    # print("Parameters: a="+str)
    rospy.logwarn("Overall score: {:0.2f}".format(last_time_steps.mean()))
    rospy.logwarn("Best 100 score: {:0.2f}".format(reduce(lambda x, y: x + y, l[-100:]) / len(l[-100:])))
    # print(".....................................................\n")

    p = subprocess.Popen("killall -9 gazebo & killall -9 gzserver  & killall -9 gzclient", shell=True)

    env.close()


