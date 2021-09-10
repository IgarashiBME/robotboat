#! /usr/bin/python
# coding:utf-8

import rospy
import numpy as np
import csv
import time

from pyproj import Proj
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelStates

# ROS custom message
from look_ahead.msg import AJK_value
from look_ahead.msg import Auto_Log
from mavlink_ajk.msg import MAV_Mission
from mavlink_ajk.msg import MAV_Modes
from ubx_analyzer.msg import NavPVT
from ubx_analyzer.msg import RELPOSNED

from tf.transformations import quaternion_from_euler
from tf.transformations import euler_from_quaternion
from tf.transformations import quaternion_multiply

SPACING = 0.8       # distance between lines 
x_tolerance = 0.1  # [meter]
yaw_tolerance = 60.0 # [Degree]
YAW_TOLERANCE_ONSTART = 30.0 # [Degree]

MAX_PIVOT_COUNT = 1

# translation value
FORWARD_CONST = 1
BACKWARD_CONST = -1

# AJK
TRANSLATION_NEUTRAL = 512     # neutral value
STEERING_NEUTRAL = 512        # neutral value
RIGHT_PIVOT = 332
LEFT_PIVOT = 692
FB_OPTIMUM = 220
LR_OPTIMUM = 60

# for simulator or test vehicle
CMD_ANGULAR_RIGHT = -0.6
CMD_ANGULAR_LEFT = 0.6
CMD_ANGULAR_LIMIT = 0.6

# frequency [Hz]
frequency = 5

# MAVLink number
MAV_CMD_NAV_WAYPOINT = 16
ARDUPILOT_AUTO_BASE = 217
ARDUPILOT_AUTO_CUSTOM = 10

# RTK fix status
FIXED_NUM = 2
FLOAT_NUM = 1

class look_ahead():
    def __init__(self):
        self.waypoint_x = []
        self.waypoint_y = []
        self.last_waypoint_x = 0
        self.last_waypoint_y = 0
        self.waypoint_seq = []
        self.waypoint_total_seq = 0
        self.x = 0
        self.y = 0
        self.q = np.empty(4)
        self.yaw = np.pi/2
        self.pre_steering_ang = 0
        self.bool_start_point = True

        # mav_modes
        self.mission_start = False
        self.base_mode = 0
        self.custom_mode = 0

        # related to GNSS
        self.iTOW = 0
        self.rtk_status = 0
        self.movingbase_status = 0
        self.latitude = 0
        self.longitude = 0

        rospy.init_node('look_ahead_following')
        rospy.on_shutdown(self.shutdown)

        # ROS callback function, receive /odom mesage
        rospy.Subscriber('/gnss_odom', Odometry, self.odom_callback, queue_size = 1)
        #rospy.Subscriber('/gazebo/model_states', ModelStates, self.truth_callback)
        rospy.Subscriber('/mav/mission', MAV_Mission, self.load_waypoint)
        rospy.Subscriber('/mav/modes', MAV_Modes, self.mav_modes)
        rospy.Subscriber('/navpvt', NavPVT, self.navpvt_callback, queue_size = 1)
        rospy.Subscriber('/relposned', RELPOSNED, self.relposned_callback, queue_size = 1)
        self.ajk_pub = rospy.Publisher('/ajk_auto', AJK_value, queue_size = 1)
        self.ajk_value = AJK_value()
        self.auto_log_pub = rospy.Publisher('/auto_log', Auto_Log, queue_size = 1)
        self.auto_log = Auto_Log()
        self.cmdvel_pub = rospy.Publisher('/roboteq_driver/cmd', Twist, queue_size = 1)
        self.sim_cmdvel_pub = rospy.Publisher('/sim_ajk/diff_drive_controller/cmd_vel', Twist, queue_size = 1)
        self.cmdvel = Twist()

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        # vehicle's quaternion data in /odom (odometry of ROS message)
        self.q[0] = msg.pose.pose.orientation.x
        self.q[1] = msg.pose.pose.orientation.y
        self.q[2] = msg.pose.pose.orientation.z
        self.q[3] = msg.pose.pose.orientation.w

    # load waypoint list
    def load_waypoint(self, msg):
        # UTM coordinate calculation
        if msg.command == MAV_CMD_NAV_WAYPOINT:
            utmzone = int((msg.longitude + 180)/6) +1   # If you are on the specific location, can't be calculated. 
            convertor = Proj(proj='utm', zone=utmzone, ellps='WGS84')
            x, y = convertor(msg.longitude, msg.latitude)
            self.last_waypoint_x = x
            self.last_waypoint_y = y
        else:
        # if not the MAV_CMD_NAV_WAYPOINT, previous waypoint will be added 
            x = self.last_waypoint_x
            y = self.last_waypoint_y
        #print x, y
        #print msg.longitude, msg.latitude

        # if msg.seq is 0, then reset variables and receive the new mission's
        if msg.seq == 0:
            self.seq = 1
            self.waypoint_x = []
            self.waypoint_y = []
            self.waypoint_seq = []
        
        self.waypoint_seq.append(msg.seq)
        self.waypoint_total_seq = msg.total_seq
        self.waypoint_x.append(x)
        self.waypoint_y.append(y) 
        #print self.waypoint_total_seq, self.waypoint_seq, self.waypoint_x, self.waypoint_y

    def mav_modes(self, msg):
        self.mission_start = msg.mission_start
        self.base_mode = msg.base_mode
        self.custom_mode = msg.custom_mode

    def navpvt_callback(self, msg):
        self.iTOW = msg.iTOW
        self.rtk_status = msg.fix_status
        self.latitude = msg.lat
        self.longitude = msg.lon

    def relposned_callback(self, msg):
        self.movingbase_status = msg.fix_status

    def cmdvel_publisher(self, steering_ang, translation, pi):
        mav_linear_velocity = rospy.get_param("/mavlink_ajk/linear_velocity")
        #rospy.loginfo(self.bool_start_point)
        if abs(steering_ang) > yaw_tolerance and self.bool_start_point == False:
            #print steering_ang
            if steering_ang >= 0:
                self.cmdvel.linear.x = 0
                self.cmdvel.angular.z = CMD_ANGULAR_LEFT
            else:
                self.cmdvel.linear.x = 0
                self.cmdvel.angular.z = CMD_ANGULAR_RIGHT
        elif self.bool_start_point == True:
            if steering_ang >= 0:
                self.cmdvel.linear.x = 0
                self.cmdvel.angular.z = CMD_ANGULAR_LEFT                   
            else:
                self.cmdvel.linear.x = 0
                self.cmdvel.angular.z = CMD_ANGULAR_RIGHT
            if abs(steering_ang) < YAW_TOLERANCE_ONSTART:
                self.bool_start_point = False
        else:
            self.cmdvel.linear.x = mav_linear_velocity*translation
            self.cmdvel.angular.z = pi

        # Angular limit
        if self.cmdvel.angular.z > CMD_ANGULAR_LIMIT:
            self.cmdvel.angular.z = CMD_ANGULAR_LIMIT
        elif self.cmdvel.angular.z < -CMD_ANGULAR_LIMIT:
            self.cmdvel.angular.z = -CMD_ANGULAR_LIMIT

        # emergency stop
        #if self.rtk_status < FIXED_NUM: #or self.movingbase_status < FIXED_NUM:
        #    self.cmdvel.linear.x = 0
        #    self.cmdvel.angular.z = 0
            
        self.cmdvel_pub.publish(self.cmdvel)
        self.sim_cmdvel_pub.publish(self.cmdvel)

    def shutdown(self):
        print "shutdown"

    def loop(self):
        rr = rospy.Rate(frequency)
        self.seq = 1
        KP = 0
        KI = 0
        KD = 0
        d = 0
        look_ahead_dist = 0
        i_control_dist = 0
        i_limit = 0
        last_iTOW = 0
        self.last_steering_ang = 0
        self.pivot_count = 0
        while not rospy.is_shutdown():
            """# if the iTOW has not increased, skip subsequent scripts 
            if self.iTOW - last_iTOW == 0:
                rospy.loginfo("warning: The value of the GNSS receiver is not updated")
                time.sleep(1)
                continue"""

            # save the last value of iTOW(GNSS time)
            last_iTOW = self.iTOW

            # if the latitude and longitude are abnormal values, skip subsequent scripts 
            if self.latitude == 0 or self.longitude == 0:
                rospy.loginfo("warning: latitude or longitude was zero")
                time.sleep(1)
                continue

            # mission checker
            if self.waypoint_total_seq != len(self.waypoint_seq) or self.waypoint_total_seq == 0:
                self.seq = 1
                rospy.loginfo("mission_checker")
                time.sleep(1)
                continue

            # mission_start checker(origin from MAV_CMD_MISSION_START)
            if self.mission_start != True or self.base_mode != ARDUPILOT_AUTO_BASE \
            or self.custom_mode != ARDUPILOT_AUTO_CUSTOM:
                # reset current_seq in mavlink_ajk_node
                self.auto_log.waypoint_seq = 0
                self.auto_log_pub.publish(self.auto_log)
                rospy.loginfo("mission_start_checker")
                time.sleep(1)
                continue

            # if a specific variable is exists, the proceeds 
            try:
                own_x = self.x
                own_y = self.y
                front_q = self.q
            except AttributeError:
                continue

            # get the parameters of look-ahead control
            KP = rospy.get_param("/mavlink_ajk/Kp")
            KI = rospy.get_param("/mavlink_ajk/Ki")
            KD = rospy.get_param("/mavlink_ajk/Kd")
            look_ahead_dist = rospy.get_param("/mavlink_ajk/look_ahead")
            i_control_dist = rospy.get_param("/mavlink_ajk/i_control_dist")
            i_limit = rospy.get_param("/mavlink_ajk/i_limit")

            # waypoint with xy coordinate origin adjust
            wp_x_adj = self.waypoint_x[self.seq] - self.waypoint_x[self.seq-1]
            wp_y_adj = self.waypoint_y[self.seq] - self.waypoint_y[self.seq-1]
            own_x_adj = own_x - self.waypoint_x[self.seq-1]
            own_y_adj = own_y - self.waypoint_y[self.seq-1]

            # coordinate transformation of waypoint
            tf_angle = np.arctan2(wp_y_adj, wp_x_adj)
            wp_x_tf = wp_x_adj*np.cos(-tf_angle) - wp_y_adj*np.sin(-tf_angle)
            wp_y_tf = wp_x_adj*np.sin(-tf_angle) + wp_y_adj*np.cos(-tf_angle)

            # coordinate transformation of own position
            own_x_tf = own_x_adj*np.cos(-tf_angle) - own_y_adj*np.sin(-tf_angle)
            own_y_tf = own_x_adj*np.sin(-tf_angle) + own_y_adj*np.cos(-tf_angle)


            # coordinate transformation of own position
            tf_q = quaternion_from_euler(0, 0, tf_angle)
            front_q_tf = quaternion_multiply((front_q[0], front_q[1], front_q[2], front_q[3]), 
                                             (   tf_q[0],    tf_q[1],    tf_q[2],   -tf_q[3]))

            # inverted
            rear_q_tf = np.empty(4)
            rear_q_tf[0] = front_q_tf[0]
            rear_q_tf[1] = front_q_tf[1]
            rear_q_tf[2] = front_q_tf[3]
            rear_q_tf[3] = -front_q_tf[2]

            # calculate the target-angle(bearing) using look-ahead distance
            bearing = np.arctan2(-own_y_tf, look_ahead_dist)
            bearing_q = quaternion_from_euler(0, 0, bearing)

            # calculate the minimal yaw error, and decide the forward or backward
            front_steering_q = quaternion_multiply(( bearing_q[0],  bearing_q[1],  bearing_q[2],  bearing_q[3]),
                                                   (front_q_tf[0], front_q_tf[1], front_q_tf[2], -front_q_tf[3]))
            rear_steering_q = quaternion_multiply((bearing_q[0], bearing_q[1], bearing_q[2], bearing_q[3]),
                                                  (rear_q_tf[0], rear_q_tf[1], rear_q_tf[2], -rear_q_tf[3]))

            front_steering_ang = euler_from_quaternion(front_steering_q)[2]/np.pi *180
            rear_steering_ang = euler_from_quaternion(rear_steering_q)[2]/np.pi *180

            steering_ang = front_steering_ang
            translation = FORWARD_CONST

            # calculate PID control value
            p = KP *steering_ang
            i = KI *own_y_tf
            if i > i_limit:
                i = i_limit
            elif i < -i_limit:
                i = -i_limit
            d = KD *(self.last_steering_ang - steering_ang)
            self.last_steering_ang = steering_ang

            pid_value = p
            if abs(own_y_tf) < i_control_dist:
                pid_value = p - i

            pid_value = pid_value - d
            
            # for simulator
            self.cmdvel_publisher(steering_ang, translation, pid_value)

            # publish autonomous log
            self.auto_log.stamp = rospy.Time.now()
            self.auto_log.iTOW = self.iTOW
            self.auto_log.rtk_status = self.rtk_status
            self.auto_log.movingbase_status = self.movingbase_status

            self.auto_log.waypoint_seq = self.seq
            self.auto_log.waypoint_start_x = self.waypoint_x[self.seq-1]
            self.auto_log.waypoint_start_y = self.waypoint_y[self.seq-1]
            self.auto_log.waypoint_end_x = self.waypoint_x[self.seq]
            self.auto_log.waypoint_end_y = self.waypoint_y[self.seq]
            self.auto_log.own_x = own_x
            self.auto_log.own_y = own_y
            self.auto_log.own_yaw = euler_from_quaternion(front_q)[2]/np.pi *180

            self.auto_log.tf_waypoint_x = wp_x_tf
            self.auto_log.tf_waypoint_y = wp_y_tf
            self.auto_log.tf_own_x = own_x_tf
            self.auto_log.tf_own_y = own_y_tf
            self.auto_log.cross_track_error = -own_y_tf

            self.auto_log.Kp = KP
            self.auto_log.Ki = KI
            self.auto_log.Kd = KD
            self.auto_log.look_ahead_dist = look_ahead_dist
            self.auto_log.i_control_dist = i_control_dist
            self.auto_log.i_limit = i_limit

            self.auto_log.p = p
            self.auto_log.i = i
            self.auto_log.d = d
            self.auto_log.steering_ang = steering_ang
            self.auto_log.linear_x = self.cmdvel.linear.x
            self.auto_log.angular_z = self.cmdvel.angular.z
            self.auto_log_pub.publish(self.auto_log)

            # when reaching the look-ahead distance, read the next waypoint.
            if (wp_x_tf - own_x_tf) < x_tolerance:
                rospy.loginfo("Next waypoint")
                pre_wp_x = self.waypoint_x[self.seq]
                pre_wp_y = self.waypoint_y[self.seq]
                self.seq = self.seq + 1
                self.bool_start_point = True # Since mission changed to the next path, the start_point flag 
                self.cmdvel.linear.x = 0
                self.cmdvel.angular.z = 0
                self.cmdvel_pub.publish(self.cmdvel)

                                             # was changed to True.
                try:
                    a = np.array([pre_wp_x, pre_wp_y])
                    b = np.array([self.waypoint_x[self.seq], self.waypoint_y[self.seq]])

                    if np.linalg.norm(a-b) < SPACING:
                        self.seq = self.seq + 1
                except IndexError:
                    pass

            if self.seq >= len(self.waypoint_x):
                self.auto_log.waypoint_seq = 65535
                self.auto_log_pub.publish(self.auto_log)

                self.cmdvel.linear.x = 0
                self.cmdvel.angular.z = 0
                self.cmdvel_pub.publish(self.cmdvel)
                self.seq = 1
                print "mission_end"
                time.sleep(5)

            rr.sleep()
  
if __name__ == '__main__':
    l = look_ahead()
    l.loop()
