# 2017.12.19 17:59:14 CST
#Embedded file name: /home/alex/Desktop/smart_wheelchair/src/limited_control_planning/src/load_map_ros.py
import rospy
from nav_msgs.msg import OccupancyGrid, MapMetaData, Odometry
from geometry_msgs.msg import Pose, PoseWithCovariance, PoseWithCovarianceStamped, PoseStamped, Point, Quaternion
from visualization_msgs.msg import Marker
from IPython import embed
import numpy as np
import tf
import time

class Mapper(object):

    def __init__(self):
        """ Start the mapper. """
        actions = ['rotation', 'translation']
        num_actions = len(actions)
        num_angs = 8
        self.the_map = np.zeros(shape=(num_actions,
         num_angs,
         400,
         400))
        self.map_resolution = 0.1
        self.x_origin = -5
        self.y_origin = -5
        self.xA = 0
        self.yA = 0
        self.thetaA = 3
        self.actionA = 0
        self.xB = 350
        self.yB = 200
        self.thetaB = 0
        self.actionB = 0
        self.goal_pub = rospy.Publisher('/goal_pub', Marker, queue_size=1)
        rospy.Subscriber('/costmap_server_node/system_costmap/costmap', OccupancyGrid, self.occupancy_grid_callback, queue_size=1)
        rospy.Subscriber('/odom', Odometry, self.odom_callback, queue_size=1)
        rospy.Subscriber('/planning_goal', PoseStamped, self.goal_callback, queue_size=1)
        rospy.Subscriber('/planning_start', PoseWithCovarianceStamped, self.start_callback, queue_size=1)

    def pub_goal(self):
        goal = Marker()
        goal.header.frame_id = '/odom'
        goal.type = goal.SPHERE
        goal.action = goal.ADD
        goal.scale.x = 0.2
        goal.scale.y = 0.2
        goal.scale.z = 0.2
        goal.color.a = 1.0
        goal.color.g = 1.0
        goal.pose.orientation.w = 1.0
        goal.pose.position.x = float(self.xB * self.map_resolution + self.x_origin)
        goal.pose.position.y = float(self.yB * self.map_resolution + self.y_origin)
        self.goal_pub.publish(goal)

    def get_map(self, scenario_name, save, load_saved):
        if save:
            np.save('/home/alex/Desktop/smart_wheelchair/src/limited_control_planning/saved_maps/' + scenario_name + '_map.npy', self.the_map)
            np.save('/home/alex/Desktop/smart_wheelchair/src/limited_control_planning/saved_maps/' + scenario_name + '_x.npy', self.xA)
            np.save('/home/alex/Desktop/smart_wheelchair/src/limited_control_planning/saved_maps/' + scenario_name + '_y.npy', self.yA)
            np.save('/home/alex/Desktop/smart_wheelchair/src/limited_control_planning/saved_maps/' + scenario_name + '_res.npy', self.map_resolution)
            print 'saved map info'
        if load_saved:
            self.the_map = np.load('/home/alex/Desktop/smart_wheelchair/src/limited_control_planning/experiments/' + scenario_name + '.npy')
            self.map_resolution = float(np.load('/home/alex/Desktop/smart_wheelchair/src/limited_control_planning/experiments/' + scenario_name + '_res.npy'))
        self.pub_goal()
        return [self.the_map,
         self.xA,
         self.yA,
         self.thetaA,
         self.actionA,
         self.xB,
         self.yB,
         self.thetaB,
         self.actionB]

    def odom_callback(self, msg):
        self.yA = int((msg.pose.pose.position.x - self.x_origin) * 1.0 / float(self.map_resolution))
        self.xA = int((msg.pose.pose.position.y - self.y_origin) * 1.0 / float(self.map_resolution))
        r, p, y = tf.transformations.euler_from_quaternion([msg.pose.pose.orientation.x,
         msg.pose.pose.orientation.y,
         msg.pose.pose.orientation.z,
         msg.pose.pose.orientation.w])
        self.thetaA = 0

    def occupancy_grid_callback(self, data):
        self.map_resolution = data.info.resolution
        self.x_origin = data.info.origin.position.x
        self.y_origin = data.info.origin.position.y
        the_map_sub = np.reshape(data.data, (400, 400))
        np.place(the_map_sub, the_map_sub > 0, 1)
        self.the_map[:, :] = the_map_sub

    def goal_callback(self, msg):
        self.xB = int((msg.pose.position.x - self.x_origin) * 1.0 / float(self.map_resolution))
        self.yB = int((msg.pose.position.y - self.y_origin) * 1.0 / float(self.map_resolution))
        self.pub_goal()

    def start_callback(self, msg):
        self.xA = int((msg.pose.pose.position.x - self.x_origin) * 1.0 / float(self.map_resolution))
        self.yA = int((msg.pose.pose.position.y - self.y_origin) * 1.0 / float(self.map_resolution))
+++ okay decompyling load_map_ros.pyc 
# decompiled 1 files: 1 okay, 0 failed, 0 verify failed
# 2017.12.19 17:59:14 CST
