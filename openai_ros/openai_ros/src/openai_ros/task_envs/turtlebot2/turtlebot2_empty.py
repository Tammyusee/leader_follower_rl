import rospy
import numpy
from gym import spaces
from openai_ros.robot_envs import turtlebot2_env
from gym.envs.registration import register
from geometry_msgs.msg import Point
from openai_ros.task_envs.task_commons import LoadYamlFileParamsTest
from openai_ros.openai_ros_common import ROSLauncher
import os

from nav_msgs.msg import Odometry


class TurtleBot2EmptyEnv(turtlebot2_env.TurtleBot2Env):
    def __init__(self):
        # This is the path where the simulation files, the Task and the Robot gits will be downloaded if not there
        ros_ws_abspath = rospy.get_param("/turtlebot2/ros_ws_abspath", None)
        assert ros_ws_abspath is not None, "You forgot to set ros_ws_abspath in your yaml file of your main RL script. Set ros_ws_abspath: \'YOUR/SIM_WS/PATH\'"
        assert os.path.exists(ros_ws_abspath), "The Simulation ROS Workspace path " + ros_ws_abspath + \
                                               " DOESNT exist, execute: mkdir -p " + ros_ws_abspath + \
                                               "/src;cd " + ros_ws_abspath + ";catkin_make"

        ROSLauncher(rospackage_name="turtlebot_gazebo",
                    launch_file_name="start_empty_world.launch",
                    ros_ws_abspath=ros_ws_abspath)

        # ROSLauncher(rospackage_name="turtlebot_gazebo",
        #             launch_file_name="put_robot2_in_world.launch",
        #             ros_ws_abspath=ros_ws_abspath)

        # Load Params from the desired Yaml file
        LoadYamlFileParamsTest(rospackage_name="openai_ros",
                               rel_path_from_package_to_file="src/openai_ros/task_envs/turtlebot2/config",
                               yaml_file_name="turtlebot2_empty.yaml")

        # Here we will add any init functions prior to starting the MyRobotEnv
        super(TurtleBot2EmptyEnv, self).__init__(ros_ws_abspath)

        # Only variable needed to be set here
        self.number_movements = rospy.get_param('/turtlebot2/n_movements')
        self.number_linear_vel = rospy.get_param('/turtlebot2/n_linear_velocity')
        self.number_angular_vel = rospy.get_param('/turtlebot2/n_angular_velocity')
        number_actions_new = self.number_movements * self.number_linear_vel * self.number_angular_vel
        self.action_space_new = spaces.Discrete(number_actions_new)

        self.combined_action_list = []
        for i in numpy.array(range(self.number_movements)):
            for j in numpy.array(range(self.number_linear_vel)):
                for k in numpy.array(range(self.number_angular_vel)):
                    self.combined_action_list.append([i, j, k])

        # self.array_action_space = numpy.array(range(len(combined_action_list)))
        # list_action_space = zip(combined_action_list, self.array_action_space)
        # print list_action_space[range of combined_action_list][1] (1 is the discretized number of action)

        ###################################################

        number_actions = rospy.get_param('/turtlebot2/n_actions')
        self.action_space = spaces.Discrete(number_actions)  # the action space will be used by qlearn.py
        # self.action_space = spaces.Discrete(number_actions_new)  # the action space will be used by qlearn.py

        # We set the reward range, which is not compulsory but here we do it.
        self.reward_range = (-numpy.inf, numpy.inf)

        # number_observations = rospy.get_param('/turtlebot2/n_observations')
        """
        We set the Observation space for the 6 observations
        cube_observations = [
            round(current_disk_roll_vel, 0),
            round(y_distance, 1),
            round(roll, 1),
            round(pitch, 1),
            round(y_linear_speed,1),
            round(yaw, 1),
        ]
        """

        # Actions and Observations
        self.linear_forward_speed = rospy.get_param('/turtlebot2/linear_forward_speed')
        self.linear_turn_speed = rospy.get_param('/turtlebot2/linear_turn_speed')
        self.angular_speed = rospy.get_param('/turtlebot2/angular_speed')
        self.init_linear_forward_speed = rospy.get_param('/turtlebot2/init_linear_forward_speed')
        self.init_linear_turn_speed = rospy.get_param('/turtlebot2/init_linear_turn_speed')

        self.new_ranges = rospy.get_param('/turtlebot2/new_ranges')
        self.min_range = rospy.get_param('/turtlebot2/min_range')
        self.max_laser_value = rospy.get_param('/turtlebot2/max_laser_value')
        self.min_laser_value = rospy.get_param('/turtlebot2/min_laser_value')

        ####################################################################
        # Get Desired Point to Get
        self.desired_point = Point()  # this might have to be distance to the leader!
        self.desired_point.x = rospy.get_param("/turtlebot2/desired_pose/x")
        self.desired_point.y = rospy.get_param("/turtlebot2/desired_pose/y")
        self.desired_point.z = rospy.get_param("/turtlebot2/desired_pose/z")

        # Get Desired Distance from the leader
        self.desired_distancde = Point()
        self.desired_distancde.x = rospy.get_param("/turtlebot2/desired_distance/x")
        self.desired_distancde.y = rospy.get_param("/turtlebot2/desired_distance/y")
        self.desired_distancde.z = rospy.get_param("/turtlebot2/desired_distance/z")

        self.distance_reward = rospy.get_param("/turtlebot2/distance_reward")
        #####################################################################

        # We create two arrays based on the binary values that will be assigned
        # In the discretization method.
        laser_scan = self.get_laser_scan()
        rospy.logdebug("laser_scan len===>" + str(len(laser_scan.ranges)))

        num_laser_readings = int(len(laser_scan.ranges)/self.new_ranges)
        high = numpy.full((num_laser_readings), self.max_laser_value)
        low = numpy.full((num_laser_readings), self.min_laser_value)

        # We only use two integers
        self.observation_space = spaces.Box(low, high)

        rospy.logdebug("ACTION SPACES TYPE===>"+str(self.action_space))
        rospy.logdebug("OBSERVATION SPACES TYPE===>"+str(self.observation_space))

        # Rewards
        self.forwards_reward = rospy.get_param("/turtlebot2/forwards_reward")
        self.turn_reward = rospy.get_param("/turtlebot2/turn_reward")
        self.end_episode_points = rospy.get_param("/turtlebot2/end_episode_points")

        self.cumulated_steps = 0.0

        self.leader_odom = Odometry()
        self.MAX_DEV = 0.2

        self.last_action = 3

    def _set_init_pose(self):
        """Sets the Robot in its init pose
        """
        self.move_base(self.init_linear_forward_speed,
                       self.init_linear_turn_speed,
                       epsilon=0.05,
                       update_rate=10)
        return True

    def _init_env_variables(self):
        """
        Inits variables needed to be initialised each time we reset at the start
        of an episode.
        :return:
        """
        # For Info Purposes
        self.cumulated_reward = 0.0
        # Set to false Done, because its calculated asyncronously
        self._episode_done = False

        odometry = self.get_odom()
        self.previous_distance_from_des_point = self.get_distance_from_desired_point(odometry.pose.pose.position)

    def _set_action(self, action):
        """
        This set action will Set the linear and angular speed of the turtlebot2
        based on the action number given.
        :param action: The action integer that set s what movement to do next.
        """
        rospy.logdebug("Start Set Action ==>"+str(action))
        # We convert the actions to speed movements to send to the parent class CubeSingleDiskEnv

        if action == 0:  # forward
            print("going forward")
            linear_speed = 0.05
            angular_speed = 0.0
            self.last_action = 0
        elif action == 1:  # left
            print("turning left")
            linear_speed = 0.1
            angular_speed = 0.5
            self.last_action = 1
        else:  # right
            print("turning right")
            linear_speed = 0.1
            angular_speed = -0.5
            self.last_action = 2

        # if self.combined_action_list[action][0] == 1:
        #     print("turning left")
        #     linear_speed = self.linear_turn_speed / (self.combined_action_list[action][2] + 1)
        #     angular_speed = -1 * self.angular_speed / (self.combined_action_list[action][2] + 1)
        # elif self.combined_action_list[action][0] == 2:
        #     print("turning right")
        #     linear_speed = self.linear_turn_speed / (self.combined_action_list[action][2] + 1)
        #     angular_speed = self.angular_speed / (self.combined_action_list[action][2] + 1)
        # elif self.combined_action_list[action][0] == 3:
        #     print("going backward")
        #     linear_speed = -1 * self.linear_forward_speed / (self.combined_action_list[action][1] + 1)
        #     angular_speed = 0
        # elif self.combined_action_list[action][0] == 4:
        #     print("stop")
        #     linear_speed = 0.0
        #     angular_speed = 0.0
        # else:
        #     print("going forward")
        #     linear_speed = self.linear_forward_speed / (self.combined_action_list[action][1] + 1)
        #     angular_speed = 0.0

        # We tell TurtleBot2 the linear and angular speed to set to execute
        self.move_base(linear_speed, angular_speed, epsilon=0.05, update_rate=10)

        rospy.logdebug("END Set Action ==>"+str(action))

    def _get_obs(self):
        """
        Here we define what sensor data defines our robots observations
        To know which Variables we have access to, we need to read the
        TurtleBot2Env API DOCS
        :return:
        """
        rospy.logdebug("Start Get Observation ==>")
        # We get the laser scan data
        laser_scan = self.get_laser_scan()

        discretized_laser_scan = self.discretize_observation(laser_scan, self.new_ranges)

        # We get the odometry so that SumitXL knows where it is.
        odometry = self.get_odom()
        x_position = odometry.pose.pose.position.x
        y_position = odometry.pose.pose.position.y

        # We round to only two decimals to avoid very big Observation space
        odometry_array = [round(x_position, 2), round(y_position, 2)]

        # We only want the X and Y position and the Yaw

        observations = discretized_laser_scan + odometry_array

        rospy.logdebug("Observations==>"+str(observations))
        rospy.logdebug("END Get Observation ==>")

        return observations

    #######################################################################################################
    def _is_done(self, observations):
        if self._episode_done:
            rospy.logerr("TurtleBot2 is Too Close to wall==>")
        else:
            rospy.logerr("TurtleBot2 didnt crash at least ==>")

            current_position = Point()
            current_position.x = observations[-2]
            current_position.y = observations[-1]
            current_position.z = 0.0

            self.leader_odom, count = self.get_leader_odom()

            desired_pose_x = self.leader_odom.pose.pose.position.x - self.desired_distancde.x
            desired_pose_y = self.desired_distancde.y

            if abs(self.leader_odom.pose.pose.position.x) >= self.leader_goal_x:
                print("LEADER HAS REACHED THE GOAL, WILL WAIT A BIT FOR THE FOLLOWER")
                if count >= 100:
                    print("THE LEADER IS DONE WAITING!!")
                    if self.is_in_desired_position(current_position):  # distance_diff <= self.desired_distancde.x + self.MAX_DEV:
                        print("A follower is IN the desired distance after a bit delayed    *1*")
                    else:
                        print("A follower is TOO FAR from the desired distance")
                    self._episode_done = True
                elif self.is_in_desired_position(current_position):
                    print("A follower is IN the desired distance    *2*")
                    self._episode_done = True

        return self._episode_done

    def _compute_reward(self, observations, done):
        current_position = Point()
        current_position.x = observations[-2]
        current_position.y = observations[-1]
        current_position.z = 0.0
        #
        # distance_from_des_point = self.get_distance_from_desired_point(current_position)
        # distance_difference = distance_from_des_point - self.previous_distance_from_des_point
        #
        # self.leader_odom, count = self.get_leader_odom()
        #
        # reward = 0
        #
        # desired_pose_x = self.leader_odom.pose.pose.position.x - self.desired_distancde.x
        # desired_pose_y = self.desired_distancde.y
        #
        # print("desired_pose_x = ", desired_pose_x)
        #
        # follower_pose_x = current_position.x
        # follower_pose_y = current_position.y
        #
        # if not done:
        #     fxfy = numpy.array((follower_pose_x, follower_pose_y))
        #     dxdy = numpy.array((desired_pose_x, desired_pose_y))
        #
        #     follower_pose_diff = numpy.linalg.norm(fxfy - dxdy)
        #     reward_discount = (follower_pose_diff * self.distance_reward) / self.desired_distancde.x
        #     reward = self.distance_reward - (reward_discount * reward_discount)
        #
        #     print("-----------------------------------------------")
        #     print("follower_pose_diff = ", follower_pose_diff)
        #     print("reward_discount = ", reward_discount)
        #     print("reward = ", reward)
        #     print("-----------------------------------------------")

        if not done:
            if self.last_action == 0:  # forward
                reward = 1
            elif self.last_action == 1:  # left
                reward = 0
            elif self.last_action == 2:  # right
                reward = 0
        else:
            if self.is_in_desired_position(current_position):
                reward = self.end_episode_points
            else:
                reward = -1 * self.end_episode_points

        print("-----------------------------------------------")
        print("reward = ", reward)
        print("-----------------------------------------------")

        # self.previous_distance_from_des_point = distance_from_des_point

        rospy.logdebug("reward=" + str(reward))
        self.cumulated_reward += reward
        rospy.logdebug("Cumulated_reward=" + str(self.cumulated_reward))
        self.cumulated_steps += 1
        rospy.logdebug("Cumulated_steps=" + str(self.cumulated_steps))

        return reward

    # Internal TaskEnv Methods

    def discretize_observation(self, data, new_ranges):  # ??????
        """
        Discards all the laser readings that are not multiple in index of new_ranges value.
        """
        self._episode_done = False

        discretized_ranges = []
        mod = len(data.ranges)/new_ranges

        rospy.logdebug("data=" + str(data))
        # rospy.logwarn("new_ranges=" + str(new_ranges))
        # rospy.logwarn("mod=" + str(mod))

        for i, item in enumerate(data.ranges):
            if i % mod == 0:
                if item == float('Inf') or numpy.isinf(item):
                    discretized_ranges.append(self.max_laser_value)
                elif numpy.isnan(item):
                    discretized_ranges.append(self.min_laser_value)
                else:
                    discretized_ranges.append(int(item))
                if self.min_range > item > 0:
                    # rospy.logerr("done Validation >>> item=" + str(item)+"< "+str(self.min_range))
                    self._episode_done = True
                # else:
                    # rospy.logwarn("NOT done Validation >>> item=" + str(item)+"< "+str(self.min_range))

        return discretized_ranges

    #######################################################################################################
    def is_in_desired_position(self, current_position, epsilon=0.05):  # add is_in_disired_radius_from_leader()
        """
        It return True if the current position is similar to the desired poistion
        """
        is_in_desired_pos = False

        if abs(self.leader_odom.pose.pose.position.x >= self.leader_goal_x):

            self.leader_odom, count_x = self.get_leader_odom()

            desired_pose_x = self.leader_odom.pose.pose.position.x - self.desired_distancde.x
            desired_pose_y = self.desired_distancde.y

            xfyf = numpy.array((current_position.x, current_position.y))
            xdyd = numpy.array((desired_pose_x, desired_pose_y))
            pose_diff = numpy.linalg.norm(xfyf - xdyd)

            if pose_diff <= self.MAX_DEV:
                is_in_desired_pos = True

            # print("pose_diff = ", pose_diff)

            # xlyl = numpy.array((self.leader_odom.pose.pose.position.x, self.leader_odom.pose.pose.position.y))
            # distance_diff = numpy.linalg.norm(xfyf - xlyl)
            # if distance_diff <= self.desired_distancde.x + self.MAX_DEV:
            #     is_in_desired_pos = True

        # x_pos_plus = self.desired_point.x + epsilon
        # x_pos_minus = self.desired_point.x - epsilon
        # y_pos_plus = self.desired_point.y + epsilon
        # y_pos_minus = self.desired_point.y - epsilon
        #
        # x_current = current_position.x
        # y_current = current_position.y
        #
        # x_pos_are_close = (x_current <= x_pos_plus) and (x_current > x_pos_minus)
        # y_pos_are_close = (y_current <= y_pos_plus) and (y_current > y_pos_minus)
        #
        # is_in_desired_pos = x_pos_are_close and y_pos_are_close

        return is_in_desired_pos
    #######################################################################################################

    def get_distance_from_desired_point(self, current_position):  # the disired point is leader's position
        """
        Calculates the distance from the current position to the desired point
        :param start_point:
        :return:
        """
        # distance = self.get_distance_from_point(current_position, self.desired_point)
        desired_odom = Point()
        desired_odom.x = 6.0 - self.desired_distancde.x  # (6.0,0.0) is the leader's goal pose
        desired_odom.y = 0.0 - self.desired_distancde.y
        distance = self.get_distance_from_point(current_position, desired_odom)

        return distance

    def get_distance_from_point(self, pstart, p_end):
        """
        Given a Vector3 Object, get distance from current position
        :param p_end:
        :return:
        """
        a = numpy.array((pstart.x, pstart.y, pstart.z))
        b = numpy.array((p_end.x, p_end.y, p_end.z))

        distance = numpy.linalg.norm(a - b)

        return distance

