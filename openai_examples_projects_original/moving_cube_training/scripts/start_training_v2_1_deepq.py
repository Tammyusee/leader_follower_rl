#!/usr/bin/env python
import os
import gym
from baselines import deepq
import rospy
import rospkg
from openai_ros.openai_ros_common import StartOpenAI_ROS_Environment


def train_deepqmodel():
    rospy.init_node('start_training_v2_1_deepq_node', anonymous=True, log_level=rospy.WARN)

    # Init OpenAI_ROS ENV: In Version 2 , there is now one system in charge of launching and downloading
    # the simulations needed for the openai_ros environment.
    task_and_robot_environment_name = rospy.get_param('/moving_cube/task_and_robot_environment_name')
    openai_ros_env_object = StartOpenAI_ROS_Environment(task_and_robot_environment_name)
    rospy.loginfo("Starting Learning")

    # Set the path where learned model will be saved
    rospack = rospkg.RosPack()
    pkg_path = rospack.get_path('moving_cube_training')
    models_dir_path = os.path.join(pkg_path, "models_saved")
    if not os.path.exists(models_dir_path):
        os.makedirs(models_dir_path)

    out_model_file_path = os.path.join(models_dir_path, "movingcube_model.pkl")

    # Load the parameters related to the training
    max_timesteps = rospy.get_param("/moving_cube/max_timesteps")
    buffer_size = rospy.get_param("/moving_cube/buffer_size")
    # We convert to float becase if we are using Ye-X notation, it sometimes treats it like a string.
    lr = float(rospy.get_param("/moving_cube/lr"))
    exploration_fraction = rospy.get_param("/moving_cube/exploration_fraction")
    exploration_final_eps = rospy.get_param("/moving_cube/exploration_final_eps")
    print_freq = rospy.get_param("/moving_cube/print_freq")

    # start the learning
    act = deepq.learn(
        openai_ros_env_object,
        network='mlp',
        lr=lr,
        total_timesteps=max_timesteps,
        buffer_size=buffer_size,
        exploration_fraction=exploration_fraction,
        exploration_final_eps=exploration_final_eps,
        print_freq=print_freq, # how many apisodes until you print total rewards and info
        param_noise=False,
        callback=None,
        load_path=None # indicate here a previous saved model (if you want to continue training it)
    )

    env.close()

    # Save the learned model
    rospy.logwarn("Saving model to movingcube_model.pkl")
    act.save(out_model_file_path)
    rospy.logwarn("Saving model to movingcube_model.pkl...DONE")

def use_trainedmodel():

    rospy.init_node('start_usingtrainedmodel_v2_1_deepq_node', anonymous=True, log_level=rospy.WARN)
    # TODO: Test and see where to place it

    # Set the path where learned model will be saved
    rospack = rospkg.RosPack()
    pkg_path = rospack.get_path('moving_cube_training')
    models_dir_path = os.path.join(pkg_path, "models_saved")
    if os.path.exists(models_dir_path):
        out_model_file_path = os.path.join(models_dir_path, "movingcube_model.pkl")

        agent.load(out_model_file_path)

        n_episodes_running = rospy.get_param('/moving_cube/episodes_running')
        agent.run(num_episodes=n_episodes_running, do_train=False)

    else:
        rospy.logerr("The path doesnt exist, please train model first==>"+str(models_dir_path))


if __name__ == '__main__':
    train_deepqmodel()