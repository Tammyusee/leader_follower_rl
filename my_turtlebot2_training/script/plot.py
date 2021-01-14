
# !/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from openai_ros.msg import RLExperimentInfo
import matplotlib.pyplot as plt


class Plot:
    # init method or constructor
    def __init__(self):
        self.x = 0
        self.y = 0
        self.ep = 0
        self.r = 0

        self.list_x = []
        self.list_y = []
        self.list_ep = []
        self.list_r = []

        self.listener()

    def start_plotting(self):
        plt.plot(self.list_ep, self.list_r)
        rospy.loginfo("\n\nPlotting...\n")
        plt.ylabel('reward')
        plt.xlabel('episode')
        plt.show()

    def listener(self):
        rospy.Subscriber("odom", Odometry, self.odom_callback)
        rospy.Subscriber("openai/reward", RLExperimentInfo, self.reward_callback)

        # plt.plot(self.list_ep, self.list_r)
        # plt.ylabel('y')
        # plt.xlabel('x')
        # plt.show()

    def odom_callback(self, data):
        self.x = data.pose.pose.position.x
        self.y = data.pose.pose.position.y
        # rospy.loginfo("received odom")
        # self.list_x.append(self.x)
        # self.list_y.append(self.y)

    def reward_callback(self, data):
        self.ep = data.episode_number
        self.r = data.episode_reward
        rospy.loginfo("received r")
        self.list_ep.append(self.ep)
        self.list_r.append(self.r)


if __name__ == '__main__':
    rospy.init_node('plotter', anonymous=True)
    pllt = Plot()
    pllt.__init__()

    while not rospy.is_shutdown():
        # rospy.loginfo("\n\nPlotting...\n")
        rospy.spin()

    pllt.start_plotting()





