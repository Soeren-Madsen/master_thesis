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
from math import sqrt
from sensor_msgs.msg import NavSatFix, Imu, Image
from kalman_ship import Kalman_est
from pose_est_board_3d import Aruco_pose
from scipy import signal
from ros_cam import Cam


'''
Denne siger hvordan PositionTarget skal sættes http://docs.ros.org/en/api/mavros_msgs/html/msg/PositionTarget.html 
HUSK: Fjern Image fra ros på rigtig drone. Bruger class Cam i stedet til at få billedet.

NÆSTE GANG: 
- Find ud af hvad jeg gør hvis jeg ikke kan finde en aruco marker.
- Få fremskaffet en bedre ground truth for at kunne lave test. Evt ent direkte model data frem. Lige nu bliver dist beregnet ud fra GPS kun, i sammenligning
og gør så de bruger samme koord sys, så man ikke skal trække random værdi fra.
- Lav samme log test som tidligere med den nye, for at se hvor godt den følger.

'''

class Drone():
    def __init__(self):
        self.f_x = open("log_x.txt", 'a')
        self.f_v = open("log_v.txt", 'a')
        self.hz = 25
        self.dist_to_target_thresh = 1 #SKAL SÆTTES HIGHER IN REAL LIFE
        self.rate = rospy.Rate(self.hz)
        self.state = State()
        self.home_position = GeoPoseStamped()
        self.current_position = PoseStamped()
        self.current_position_geo = GeoPoseStamped()
        self.current_imu = Imu()
        self.receivedPosition = False
        #self.home_position = NavSatFix()
        #self.current_position = NavSatFix()
        self.altitude = 4.
        self.uc = utmconv()
        self.setup_topics()
        self.kf = Kalman_est()
        self.kf.init_guess(self.altitude)
        self.landing_allowed = False
        self.image_ac = False
        self.allow_landing = False
        self.counter = 0
        self.max_alt_ship = 0
        self.landing_speed = 1.5

        self.aru = Aruco_pose()

        self.cam = Cam()

        #Lowpass filter:
        self.b = signal.firwin(25, 0.02)
        self.z = signal.lfilter_zi(self.b, 1)*0 #0 er start værdi for filteret


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
        self.target_pos_sub = rospy.Subscriber("/fix", NavSatFix, self.target_cb)
        self.imu_sub = rospy.Subscriber('/mavros/imu/data', Imu, self.imu_cb)
        self.img_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.image_cb)

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
        #print("longitude %f" % self.targetGPS.longitude)
        self.receivedGPS = True

    def imu_cb(self, data):
        self.current_imu = data

    def image_cb(self, img_msg):
        # log some info about the image topic
        

        # Try to convert the ROS Image message to a CV2 Image
        self.cv_image = np.frombuffer(img_msg.data, dtype=np.uint8).reshape(img_msg.height, img_msg.width, -1)
        # Show the converted image

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
        self.takeOffPosition.pose.position.y= self.current_position.pose.position.y
        self.takeOffPosition.pose.position.z = 0

        # send a few takeoff commands before starting
        for i in range(20):
            self.target_pos_pub.publish(self.takeOffPosition)
            self.rate.sleep()


        print("Waiting for change mode to offboard")
        print("Enabling OFFBOARD")
        while not self.state.mode == "OFFBOARD": #FJERN DETTE I REAL LIFE
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

            # reached takeoff position
        header = Header()
        dist_to_takeoff_pos = 99999
        self.takeOffPosition.pose.position.z = self.altitude
        while(self.dist_to_target_thresh < dist_to_takeoff_pos):
            x = self.takeOffPosition.pose.position.x - self.current_position.pose.position.x
            y = self.takeOffPosition.pose.position.y - self.current_position.pose.position.y
            z = self.takeOffPosition.pose.position.z - self.current_position.pose.position.z
            dist_to_takeoff_pos = sqrt(x*x + y*y + z*z)
            header.stamp = rospy.Time.now()
            #print(self.takeOffPosition.pose.position.altitude)
            #print(self.current_position.altitude)
            self.takeOffPosition.header = header
            self.target_pos_pub.publish(self.takeOffPosition)
            self.rate.sleep()

    def shutdownDrone(self):
        while not self.state.mode == "AUTO.LAND":
            self.land_client()
            print("Landing..")
        rospy.signal_shutdown("Done")
    
    def set_target(self):
        targetPosition = PoseStamped()
        targetPosition.pose.position.x = -50
        targetPosition.pose.position.y = 0
        targetPosition.pose.position.z = self.targetGPS.altitude - self.current_position_geo.altitude + self.altitude
        return targetPosition

    def follow_ship(self):

        #self.cv_image = self.cam.get_img() !!!!!!!!!!!!!!SKAL BRUGES PÅ DRONEN

        start = time.time()
        targetPosition = PositionTarget()
        targetPosition.coordinate_frame = 1
        #targetPosition = PoseStamped()
        targetPosition.type_mask = 4088 #Ignore everything but PX, PY, PZ, #VZ #4056
        self.kf.update_control_sig(self.current_imu.linear_acceleration.z-9.8)
        #targetPosition.pose.position.x = -50.
        #targetPosition.pose.position.y = 0.

        targetPosition.position.x = -50
        targetPosition.position.y = 0
        dist =  self.current_position_geo.altitude - self.targetGPS.altitude-0.5
        dist_aruco = self.aru.calc_euler(self.cv_image) #Calculate distance to ship based on aruco markers
        #print("Distance to ship: ", dist)
        print("Distance to ship aruco: ", dist_aruco[0])
        #print("Dist dif: ", dist_aruco[0] - dist)
        #print("Altitude of drone: ", self.current_position.pose.position.z)
        x = self.kf.update_predict(dist_aruco[0], self.current_position.pose.position.z)
        
        #targetPosition.pose.position.z = x[2] + self.altitude
        targetPosition.position.z = x[2] + self.altitude

        st, self.z = signal.lfilter(self.b, 1, [x[3]], zi=self.z)
        self.f_v.write(','.join([str(self.current_position.pose.position.z), str(self.targetGPS.altitude - self.home_alt + 5.35), str(dist_aruco[0]), str(dist), str(x[1]), '\n']))
        #self.f_v.write(',')

        #targetPosition.velocity.z = x[3] #No low pass filter
        targetPosition.velocity.z = st[0] #With low pass filter

        self.f_x.write(','.join([str(x[0]), str(x[1]), str(x[2]), str(x[3]), '\n']))

        ship_alt = self.targetGPS.altitude - self.home_alt + 5.35
        if dist_aruco < self.altitude + 0.2:
            self.counter = self.counter + 1
            if ship_alt<self.max_alt_ship:
                self.max_alt_ship = ship_alt
        print("ship min alt: ", self.max_alt_ship)
        print("ship alt: ", ship_alt)
        
        
        if self.counter > 200 and ship_alt < 0.9*self.max_alt_ship:
            self.allow_landing = True
            

        if self.allow_landing:
            self.altitude = self.altitude - self.landing_speed/self.hz
            print("Landing")
            

        end = time.time()
        #print("Time taken: ", end-start)
        self.rate.sleep()
        
        #if not self.image_ac and dist_aruco < 5.1:
        #    cv2.imwrite('test.jpg', self.cv_image)
        #    image_ac = True
        #self.f_x.write(',')

        

        return targetPosition#, targetVelocity
        

    def distanceToTarget(self,targetPosition):
        x = targetPosition.pose.position.x - self.current_position.pose.position.x
        y = targetPosition.pose.position.y - self.current_position.pose.position.y
        z = targetPosition.pose.position.z - self.current_position.pose.position.z
        return sqrt(x*x + y*y + z*z)

    def fly_to_ship(self):
        header = Header()
        while (True):
            header.stamp = rospy.Time.now()
            if self.landing_allowed:
                     
                #target_pos, target_vel = self.follow_ship()
                target_pos = self.follow_ship()
                target_pos.header = header
                #target_vel.header = header
                #print("target_pos: ", target_pos.pose.position.z)
                #print("target_vel: ", target_vel)            
                #self.target_vel_pub.publish(target_vel)
                #self.target_pos_pub.publish(target_pos)

                self.target_pos_pub2.publish(target_pos)
                
            else:
                target_pos = self.set_target()
                target_pos.header = header
                self.target_pos_pub.publish(target_pos)
                print(self.distanceToTarget(target_pos))
                if(self.distanceToTarget(target_pos) < self.dist_to_target_thresh):
                    self.landing_allowed = True

            self.rate.sleep()

if __name__ == '__main__':
    rospy.init_node('drone_control', anonymous=True)
    drone = Drone()
    drone.setup()
    drone.fly_to_ship()
    drone.shutdownDrone()
    rospy.spin()