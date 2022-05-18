#!/usr/bin/env python3

import math
import rospy
import mavros
import cv2
import numpy as np
import time
import sys
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
from trajectory import Trajec

class Drone():
    def __init__(self, arg = False):
        self.gazebo = arg

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
        if self.gazebo:
            self.altitude = 2. #4
            self.dist_to_target_thresh = 0.5
        else:
            self.altitude = 3.5
            self.dist_to_target_thresh = 0.5
        self.uc = utmconv()
        self.setup_topics()
        self.counter = 0
        self.start_time = -1

        if self.gazebo:
            self.x_pos = 0
        else:
            self.x_pos = 0
        self.y_pos = 0

        self.aru = Aruco_pose(self.gazebo)

        self.cam = Cam()

        self.traj = Trajec(self.altitude)
        self.start_traj = False
        self.motion_old = False
        self.mot_old = 0
        self.time_old = time.time()
        self.t_last = 0


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
        self.motion_table_sub = rospy.Subscriber("/vrpn_client_node/motion_table/pose", PoseStamped, self.motion_table_cb)

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

    def motion_table_cb(self, position):
        self.motion_table = position
        if not self.motion_old:
            self.mot_old = position.pose.position.z
            self.motion_old = True

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
        while not self.state.mode == "OFFBOARD":
            self.rate.sleep()
            self.target_pos_pub.publish(self.takeOffPosition)

        print("Rotorcraft arming")
        while not self.state.armed:
            self.target_pos_pub.publish(self.takeOffPosition)
            self.rate.sleep()
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
            print("Distance to takeoff: ", dist_to_takeoff_pos)
            print("Drone z: ", self.current_position.pose.position.z)
            self.rate.sleep()
        
        dist_to_takeoff_pos = 99999
        self.takeOffPosition.pose.position.z = self.altitude
        self.takeOffPosition.pose.position.x = self.motion_table.pose.position.x
        self.takeOffPosition.pose.position.y = self.motion_table.pose.position.y
        self.counter=0        
        while self.counter < 300 or self.dist_to_target_thresh < dist_to_takeoff_pos:
            if self.start_time < 0:
                self.start_time = time.time()
            dist_to_takeoff_pos = self.distanceToTarget(self.takeOffPosition)
            header.stamp = rospy.Time.now()
            self.takeOffPosition.header = header
            self.target_pos_pub.publish(self.takeOffPosition)
            self.counter = self.counter + 1
            self.rate.sleep()
        
        dist_to_takeoff_pos = 99999
        targetPosition = PositionTarget()
        targetPosition.coordinate_frame = 1
        targetPosition.type_mask = 3064 #Ignore everything but PX, PY, PZ, VZ #4088. 3064 er med Yaw
        print("start")
        while True:
            if not self.start_traj:
                if abs(self.motion_old - self.motion_table.pose.position.z) > 0.02:
                    self.start_traj = True
                    self.time_old = time.time()
                    self.t_last = 0

            if self.start_traj:
                t = time.time() - self.time_old
                dt = t-self.t_last
                zr,zd_ref = self.traj.solve_numeric(t, dt)
                self.t_last = t

            
            targetPosition.pose.position.x = self.motion_table.pose.position.x
            targetPosition.pose.position.y = self.motion_table.pose.position.y
            targetPosition.pose.position.z = self.altitude

            targetPosition.velocity.z = 0
            header.stamp = rospy.Time.now()
            self.takeOffPosition.header = header
            self.target_pos_pub.publish(self.takeOffPosition)
            self.f_v.write(','.join([str(zr),str(self.current_position.pose.position.z), str(time.time()- self.start_time), str(self.motion_table.pose.position.z), str(zd_ref), '\n'])) #Z dist
            self.rate.sleep()
        

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
    try:
        arg = sys.argv[1]
        if arg == "Gazebo":
            arg = True
            print("Running Gazebo!!!!!!!!!!!!!!!!!!!!!!!!!!!")
        else:
            arg = False
    except:
        print("Running optitrack")
        arg = False
    
    print(arg)
    drone = Drone(arg)
    drone.setup()
    #drone.fly_to_ship()
    drone.shutdownDrone()
    rospy.spin()
