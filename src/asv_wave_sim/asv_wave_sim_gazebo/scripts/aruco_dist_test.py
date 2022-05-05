#!/usr/bin/env python3

import rospy
import mavros
import cv2
import numpy as np
import time
from utm import utmconv
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped, TwistStamped, Twist
from geographic_msgs.msg import GeoPoseStamped
from mavros_msgs.msg import State, PositionTarget
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
from math import sqrt, degrees, radians, fmod
from sensor_msgs.msg import NavSatFix, Imu, Image
from pose_est_board_3d import Aruco_pose
from ros_cam import Cam

class Drone():
    def __init__(self):
        self.gazebo = True

        self.f_x = open("log_x.txt", 'a')
        self.f_v = open("log_v.txt", 'a')
        self.hz = 25
        self.rate = rospy.Rate(self.hz)
        self.state = State()
        self.home_position = GeoPoseStamped()
        self.current_position = PoseStamped()
        self.current_position_geo = GeoPoseStamped()
        self.current_imu = Imu()
        self.receivedPosition = False
        #self.home_position = NavSatFix()
        #self.current_position = NavSatFix()
        if self.gazebo:
            self.altitude = 6. #4
            self.dist_to_target_thresh = 1
        else:
            self.altitude = 3.
            self.dist_to_target_thresh = 2
        self.uc = utmconv()
        self.setup_topics()
        self.landing_allowed = False
        self.image_ac = False
        self.allow_landing = False
        self.counter = 0
        self.max_alt_ship = -10
        if self.gazebo:
            self.landing_speed = 1.5
        else:
            self.landing_speed = 1
        self.yaw_fixed = False
        self.alt_flag = False
        self.last_ship_alt = 0
        self.last_dist = 99
        self.start_time = -1
        self.old_time = -1
        self.last_roll = -19

        if self.gazebo:
            self.x_pos = -50
        else:
            self.x_pos = 0
        self.y_pos = 0
        self.key_press = 0

        self.aru = Aruco_pose(self.gazebo)

        self.cam = Cam()



    def setup_topics(self):
        rospy.wait_for_service("/mavros/cmd/arming")
        rospy.loginfo("/mavros/cmd/arming service ready!")
        rospy.wait_for_service("mavros/set_mode")
        rospy.loginfo("/mavros/set_mode service ready!")
        rospy.wait_for_service("mavros/cmd/land")
        rospy.loginfo("/mavros/cmd/land service ready!")
        self.arming_client = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
        self.set_mode_client = rospy.ServiceProxy("/mavros/set_mode", SetMode)
        self.land_client = rospy.ServiceProxy("/mavros/cmd/land", CommandTOL)
        self.takeoff_client = rospy.ServiceProxy("/mavros/cmd/takeoff", CommandTOL)


        ## Subscribers:
        self.state_sub = rospy.Subscriber('/mavros/state', State, self.state_cb)
        self.pos_sub = rospy.Subscriber("/mavros/global_position/raw/fix", NavSatFix, self.gps_pos_cb)
        self.pos_sub = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.position_cb)
        self.imu_sub = rospy.Subscriber('/mavros/imu/data', Imu, self.imu_cb)

        ## Publishers:
        self.target_pos_pub = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size=1)
        self.target_pos_pub2 = rospy.Publisher("/mavros/setpoint_raw/local", PositionTarget, queue_size=1)
        self.target_vel_pub = rospy.Publisher("/mavros/setpoint_velocity/cmd_vel", TwistStamped, queue_size=1)
        #self.target_pos_pub = rospy.Publisher("/mavros/setpoint_position/global", GeoPoseStamped, queue_size=1)

    def state_cb(self, state):
        self.state = state

    def gps_pos_cb(self, position):
        self.current_position_geo = position
        if(not self.receivedPosition):
            self.home_position = position
            (self.hemisphere, self.zone, _, self.home_e, self.home_n) = self.uc.geodetic_to_utm (self.home_position.latitude,self.home_position.longitude)
            self.home_e = self.home_e - self.current_position.pose.position.x
            self.home_n = self.home_n - self.current_position.pose.position.y
            self.home_alt = self.current_position_geo.altitude
        self.receivedPosition = True

    def position_cb(self, position):
        self.current_position = position

    def target_cb(self, data):
        self.targetGPS = data
        self.receivedGPS = True

    def imu_cb(self, data):
        self.current_imu = data

    def image_cb(self, img_msg):
        # Try to convert the ROS Image message to a CV2 Image
        self.cv_image = np.frombuffer(img_msg.data, dtype=np.uint8).reshape(img_msg.height, img_msg.width, -1)


    def setup(self):
        print("Waiting for FCU connection...")
        while not self.state.connected:
            self.rate.sleep()
        print("FCU connected")

        print("Waiting on position...")
        while not self.receivedPosition:
            self.rate.sleep()
        print("Position received")

        
        self.takeOffPosition = PoseStamped()
        self.takeOffPosition.pose.position.x = self.current_position.pose.position.x
        self.takeOffPosition.pose.position.y = self.current_position.pose.position.y
        self.takeOffPosition.pose.position.z = 0

        # send a few takeoff commands before starting
        for i in range(20):
            self.target_pos_pub.publish(self.takeOffPosition)
            self.rate.sleep()


        print("Waiting for change mode to offboard")
        if self.gazebo:
            print("Enabling OFFBOARD")
            while not self.state.mode == "OFFBOARD":
                self.rate.sleep()
                self.target_pos_pub.publish(self.takeOffPosition)
                if not self.state.mode == "OFFBOARD":
                    self.set_mode_client(base_mode=0, custom_mode="OFFBOARD")
                    print("OFFBOARD enable")

        print("Rotorcraft arming")
        while not self.state.armed:
            self.target_pos_pub.publish(self.takeOffPosition)
            if not self.state.armed:
                self.arming_client(True)

        header = Header()
        dist_to_takeoff_pos = 99999
        self.takeOffPosition.pose.position.z = self.altitude
        while(self.dist_to_target_thresh < dist_to_takeoff_pos):
            dist_to_takeoff_pos = self.distanceToTarget(self.takeOffPosition)
            header.stamp = rospy.Time.now()
            self.takeOffPosition.header = header
            self.target_pos_pub.publish(self.takeOffPosition)
            self.rate.sleep()
        
        dist_to_takeoff_pos = 99999
        self.takeOffPosition.pose.position.x = 0
        self.takeOffPosition.pose.position.y = 0
        self.takeOffPosition.pose.position.z = self.altitude
        
        while(self.dist_to_target_thresh < dist_to_takeoff_pos):
            if self.start_time < 0:
                self.start_time = time.time()
            dist_to_takeoff_pos = self.distanceToTarget(self.takeOffPosition)
            header.stamp = rospy.Time.now()
            self.takeOffPosition.header = header
            self.target_pos_pub.publish(self.takeOffPosition)
            self.cv_image = self.cam.get_img()
            success, dist_aruco, yaw, y_cor, x_cor, roll, pitch = self.aru.calc_euler(self.cv_image)
            self.f_v.write(','.join([str(dist_aruco),str(self.current_position.pose.position.z), str(time.time()- self.start_time), '\n']))
            self.rate.sleep()

        dist_to_takeoff_pos = 99999
        self.takeOffPosition.pose.position.z = self.altitude-1
        while self.counter < 300 and self.dist_to_target_thresh < dist_to_takeoff_pos:
            self.cv_image = self.cam.get_img()
            success, dist_aruco, yaw, y_cor, x_cor, roll, pitch = self.aru.calc_euler(self.cv_image)
            self.f_v.write(','.join([str(dist_aruco),str(self.current_position.pose.position.z), str(time.time()- self.start_time), '\n']))
            self.counter = self.counter + 1

        dist_to_takeoff_pos = 99999
        self.counter = 0
        self.takeOffPosition.pose.position.z = self.altitude
        while self.counter < 300 and self.dist_to_target_thresh < dist_to_takeoff_pos:
            self.cv_image = self.cam.get_img()
            success, dist_aruco, yaw, y_cor, x_cor, roll, pitch = self.aru.calc_euler(self.cv_image)
            self.f_v.write(','.join([str(dist_aruco),str(self.current_position.pose.position.z), str(time.time()- self.start_time), '\n']))
            self.counter = self.counter + 1

    def shutdownDrone(self):
        while not self.state.mode == "AUTO.LAND":
            self.land_client()
            print("Landing..")
        rospy.signal_shutdown("Done")

    def distanceToTarget(self,targetPosition):
        x = targetPosition.pose.position.x - self.current_position.pose.position.x
        y = targetPosition.pose.position.y - self.current_position.pose.position.y
        z = targetPosition.pose.position.z - self.current_position.pose.position.z
        return sqrt(x*x + y*y + z*z)

if __name__ == '__main__':
    rospy.init_node('drone_control', anonymous=True)
    drone = Drone()
    drone.setup()
    drone.fly_to_ship()
    drone.shutdownDrone()
    rospy.spin()