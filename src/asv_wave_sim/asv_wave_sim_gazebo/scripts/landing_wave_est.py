#!/usr/bin/env python3

import rospy
import mavros
import cv2
import numpy as np
import time
import math
from utm import utmconv
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped, TwistStamped, Twist, Vector3Stamped
from geographic_msgs.msg import GeoPoseStamped
from mavros_msgs.msg import State, PositionTarget
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
from sensor_msgs.msg import NavSatFix, Imu, Image
from kalman_ship import Kalman_est
from pose_est_board_3d import Aruco_pose
from scipy import signal
from ros_cam import Cam
from gazebo_msgs.srv import GetModelState, SetModelState, SetLinkState
from gazebo_msgs.msg import ModelState, LinkState


class Drone():
    def __init__(self):
        self.gazebo = True

        self.f_x = open("log_x.txt", 'a')
        self.f_v = open("log_v.txt", 'a')
        self.f_a = open("log_vel_kalman.txt", 'a')
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
            self.altitude = 5. #4
            self.dist_to_target_thresh = 1
            self.landing_speed = 2
            self.x_pos = -50
        else:
            self.altitude = 3.
            self.dist_to_target_thresh = 2
            self.landing_speed = 1
            self.x_pos = 0
        self.uc = utmconv()
        self.setup_topics()
        self.kf = Kalman_est()
        self.kf.init_guess(self.altitude)
        self.landing_allowed = False
        self.image_ac = False
        self.allow_landing = False
        self.counter = 0
        self.max_alt_ship = -10
        self.yaw_fixed = False
        self.alt_flag = False
        self.last_ship_alt = 0
        self.last_dist = 99
        self.start_time = -1
        self.old_time = -1
        self.last_roll = -19
        self.y_pos = 0
        self.key_press = 0

        self.aru = Aruco_pose(self.gazebo)

        self.cam = Cam()

        #Lowpass filter:
        self.b = signal.firwin(25, 0.02)
        self.z = signal.lfilter_zi(self.b, 1)*0 #0 is start value of filter
        self.b2,self.a2 = signal.butter(2, 0.35, fs=10)
        self.z2 = signal.lfilter_zi(self.b2, self.a2)*3.1415 #3.1415 is start value of filter
        self.b3,self.a3 = signal.butter(2, 0.35, fs=10)
        self.z3 = signal.lfilter_zi(self.b3, self.a3)*0 #0 is start value of filter


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
        if self.gazebo:
            self.img_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.image_cb)
            self.set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
            self.set_link = rospy.ServiceProxy('/gazebo/set_link_state', SetLinkState)
            self.ship_vel_sub = rospy.Subscriber("/fix_velocity", Vector3Stamped, self.ship_cb)
            self.drone_vel_sub = rospy.Subscriber("/mavros/local_position/velocity_local", TwistStamped, self.drone_vel_cb)

        ## Publishers:
        self.target_pos_pub = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size=1)
        self.target_pos_pub2 = rospy.Publisher("/mavros/setpoint_raw/local", PositionTarget, queue_size=1)

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

    def ship_cb(self, data):
        self.ship_vel = data

    def drone_vel_cb(self, data):
        self.drone_vel = data


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
            #if self.gazebo:
                #self.reset_link(0, 0)
            x = self.takeOffPosition.pose.position.x - self.current_position.pose.position.x
            y = self.takeOffPosition.pose.position.y - self.current_position.pose.position.y
            z = self.takeOffPosition.pose.position.z - self.current_position.pose.position.z
            dist_to_takeoff_pos = math.sqrt(x*x + y*y + z*z)
            header.stamp = rospy.Time.now()
            self.takeOffPosition.header = header
            self.target_pos_pub.publish(self.takeOffPosition)
            self.rate.sleep()

    def shutdown_drone(self):
        while not self.state.mode == "AUTO.LAND":
            self.land_client()
            print("Landing..")
        rospy.signal_shutdown("Done")
    
    def set_target(self):
        targetPosition = PoseStamped()
        targetPosition.pose.position.x = self.x_pos
        targetPosition.pose.position.y = self.y_pos
        if self.gazebo:
            targetPosition.pose.position.z = self.targetGPS.altitude - self.current_position_geo.altitude + self.altitude
        else:
            targetPosition.pose.position.z = self.altitude
        return targetPosition

    def follow_ship(self):
        start = time.time()
        if self.gazebo:
            self.model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
            model_ship = self.model_state("ship", "")
            model_drone = self.model_state("sdu_master", "")
            #print("Drone z pos: ", model_drone.pose.position.z)
            #print("ship z pos: ", model_ship.pose.position.z)

        if not self.gazebo:
            self.cv_image = self.cam.get_img()

        start = time.time()
        targetPosition = PositionTarget()
        targetPosition.coordinate_frame = 1
        #targetPosition = PoseStamped()
        targetPosition.type_mask = 3064 #Ignore everything but PX, PY, PZ, VZ #4088. 3064 er med Yaw
        #self.kf.update_control_sig(self.current_imu.linear_acceleration.z-9.8)
        #targetPosition.pose.position.x = -50.
        #targetPosition.pose.position.y = 0.

        
        success, dist_aruco, yaw, y_cor, x_cor, roll, pitch = self.aru.calc_euler(self.cv_image) #Calculate distance to ship based on aruco markers
        if self.gazebo:
            if success:
                drone_yaw = self.aru.euler_from_quaternion(self.current_position.pose.orientation.x, self.current_position.pose.orientation.y, self.current_position.pose.orientation.z, self.current_position.pose.orientation.w)[2]
                #self.x_pos = self.current_position.pose.position.x + y_cor*math.cos(drone_yaw)*1 + x_cor*math.sin(drone_yaw)*1 #Changed for gazebo coordinate system
                #self.y_pos = self.current_position.pose.position.y - y_cor*math.sin(drone_yaw)*1 + x_cor*math.cos(drone_yaw)*1 #Changed for gazebo coordinate system with opposite y axis
                #print("Yaw: ", drone_yaw)
                #print("X pos: ", self.x_pos)
            dist =  model_drone.pose.position.z - model_ship.pose.position.z
            #targetPosition.position.x = model_ship.pose.position.x-50
            #targetPosition.position.y = model_ship.pose.position.x
            targetPosition.position.x = self.x_pos
            targetPosition.position.y = self.y_pos
        #else:
            #targetPosition.position.x = self.x_pos
            #targetPosition.position.y = self.y_pos
        
        #if success and self.last_dist > 1.5:
        
        #print("yaw: ", yaw)
        #if abs(yaw) < 5 or self.yaw_fixed: 
        self.yaw_fixed = True
        drone_yaw = self.aru.euler_from_quaternion(self.current_position.pose.orientation.x, self.current_position.pose.orientation.y, self.current_position.pose.orientation.z, self.current_position.pose.orientation.w)[2]
        targ = math.fmod(drone_yaw - math.radians(yaw), 3.1415)
        #targetPosition.yaw = targ
        #if not self.gazebo and not self.alt_flag:
            #self.altitude = 2
            #self.alt_flag = True
        #print("Distance to ship: ", dist)
        self.last_dist = dist
        #print("Distance to ship aruco: ", dist_aruco[0])
        #print("Dist dif: ", dist_aruco[0] - dist)
        #print("Altitude of drone: ", self.current_position.pose.position.z)
        #x = self.kf.update_predict(dist_aruco[0], self.current_position.pose.position.z)
        #x = self.kf.update_predict(dist, self.current_position.pose.position.z)
        x = self.kf.future_predict(dist, self.current_position.pose.position.z) #Prediction in the future
        sdf = self.kf.update_predict(dist, self.current_position.pose.position.z)
        if self.start_time < 0:
            self.start_time = time.time()
        self.kf.check_time()
        
        #targetPosition.pose.position.z = x[2] + self.altitude
        targetPosition.position.z = x[2] + self.altitude
        #targetPosition.pose.position.z = self.altitude
        #print("Kalman vel: ", x[3])
        #print("ship vel: ", model_ship.twist.linear.z)

        st, self.z = signal.lfilter(self.b, 1, [x[3]], zi=self.z)
        
        #targetPosition.velocity.z = x[3] #No low pass filter
        targetPosition.velocity.z = st[0] #With low pass filter
        self.last_ship_alt = x[2]

        #self.f_x.write(','.join([str(x[0]), str(x[1]), str(x[2]), str(x[3]), str(x[4]), '\n']))
        if self.gazebo:
            ship_alt = model_ship.pose.position.z
        else:
            ship_alt = 0 #ÆNDRER TIL ALT AF MOTIONTABLE
        if dist < self.altitude + 0.5:
            self.counter = self.counter + 1
            if ship_alt<self.max_alt_ship:
                self.max_alt_ship = ship_alt
        #print("ship min alt: ", self.max_alt_ship)
        #print("ship alt: ", ship_alt)
        if roll < 0:
            roll = roll + 3.1416*2

        roll_lp, self.z2 = signal.lfilter(self.b2, self.a2, [roll], zi=self.z2)
        pitch_lp, self.z3 = signal.lfilter(self.b3, self.a3, [pitch], zi=self.z3)
        #print("Roll lp: {} pitch lp: {}".format(roll_lp,pitch_lp))

        if self.old_time < 0:
            self.old_time = time.time()
        new_time = time.time()
        #if new_time - self.old_time > 5:
            #if self.gazebo:
                #self.reset_link(roll_lp[0]-3.1415, pitch_lp[0])
                #print("Reset link")
        #else:
            #self.reset_link(0, 0)
        #self.f_v.write(','.join([str(model_drone.pose.position.z), str(model_ship.pose.position.z), str(dist_aruco[0]), str(time.time()- self.start_time), str(x[1]), '\n']))
        roll_gt, pitch_gt, yaw_gt = self.aru.euler_from_quaternion(model_ship.pose.orientation.x, model_ship.pose.orientation.y, model_ship.pose.orientation.z, model_ship.pose.orientation.w)
        roll_drone, pitch_drone, yaw_drone = self.aru.euler_from_quaternion(model_drone.pose.orientation.x, model_drone.pose.orientation.y, model_drone.pose.orientation.z, model_drone.pose.orientation.w)
        #self.f_v.write(','.join([str(roll_gt), str(roll_lp[0]), str(roll), str(time.time()- self.start_time), '\n']))
        #self.f_v.write(','.join([str(dist), str(dist_aruco[0]), str(time.time()- self.start_time), '\n']))
        #self.f_a.write(','.join([str(dist_aruco[0]),str(self.current_position.pose.position.z), str(time.time()- self.start_time), str(model_ship.pose.position.z), '\n'])) #Z dist
        self.f_x.write(','.join([str(x[0]),str(x[1]), str(time.time()- self.start_time), str(x[2]), str(x[3]), str(x[4]), '\n'])) #kalman
        if self.counter > 200: #and ship_alt < 0.9*self.max_alt_ship:
            self.allow_landing = True
        #print("Ship vel: ", self.ship_vel.vector.z)
        
        end = time.time()
        #print("time: ", end-start)
        if self.allow_landing:
            self.altitude = self.altitude - self.landing_speed/self.hz
            if dist < 1.25 and not self.alt_flag:
                self.alt_flag = True
                print("logging")
                self.f_a.write(str(self.ship_vel.vector.z))
                self.f_a.write(',')
                self.f_a.write(str(self.drone_vel.twist.linear.z))
                self.f_a.write(',')
                self.f_a.write(str(self.drone_vel.twist.linear.z - self.ship_vel.vector.z))
                self.f_a.write('\n')
                #self.f_a.write("logging")
                self.f_a.close()
                
                #self.f_a.write('\n')
            #print("Landing")
            #print("IMU value: ", self.current_imu.linear_acceleration.z)
            #self.f_a.write(str(self.current_imu.linear_acceleration.z))
            #self.f_a.write(',')
            #self.f_a.write('\n')
        # else:
        #     drone_yaw = self.aru.euler_from_quaternion(self.current_position.pose.orientation.x, self.current_position.pose.orientation.y, self.current_position.pose.orientation.z, self.current_position.pose.orientation.w)[2]
        #     targ = math.fmod(drone_yaw - math.radians(yaw), 3.1415)
        #     #targetPosition.yaw = targ
        #     if self.gazebo:
        #         self.reset_link(0, 0)
        #     print("target yaw: ", targ)
        #     print("Drone yaw: ", drone_yaw)
            
        # else:
            
        #     if self.allow_landing:
        #         self.altitude = self.altitude - self.landing_speed/self.hz
        #         print("Landing no Aruco")
        #         print("IMU value: ", self.current_imu.linear_acceleration.z)
        #         self.f_a.write(str(self.current_imu.linear_acceleration.z))
        #         self.f_a.write(',')
        #     else:
        #         print("Unable to find aruco marker")
        #     #targetPosition.position.z = self.last_ship_alt + self.altitude
        #     #targetPosition.pose.position.z = self.last_ship_alt + self.altitude
        #     targetPosition.pose.position.z =self.altitude
        #     drone_yaw = self.aru.euler_from_quaternion(self.current_position.pose.orientation.x, self.current_position.pose.orientation.y, self.current_position.pose.orientation.z, self.current_position.pose.orientation.w)[2]
        #     targ = math.fmod(drone_yaw - math.radians(yaw), 3.1415)
        #     #targetPosition.yaw = targ 
        
        end = time.time()
        #print("Time taken: ", end-start)
        self.rate.sleep()
        

        

        return targetPosition
        
    def landing_gear_move(self, roll, pitch, yaw):
        state_msg = ModelState()
        state_msg.model_name = 'sdu_master::landing'
        #state_msg.pose.position.x = 0
        #state_msg.pose.position.y = 0
        #state_msg.pose.position.z = 0.3
        roll = roll
        pitch = pitch
        yaw = yaw
        ori = self.quaternion_from_euler(roll, pitch, yaw)
        state_msg.pose.orientation.x = ori[0]
        state_msg.pose.orientation.y = ori[1]
        state_msg.pose.orientation.z = ori[2]
        state_msg.pose.orientation.w = ori[3]
        state_msg.reference_frame = 'sdu_master'
        #print("waiting for model state")
        rospy.wait_for_service('/gazebo/set_model_state')
        #print("Changing state")
        resp = self.set_state(state_msg)

    def reset_link(self, roll, pitch):
        ori = self.quaternion_from_euler(pitch, -roll, 0.0)
        link_msg = LinkState()
        link_msg.link_name = 'sdu_master::landing::landing_gear'
        #link_msg.pose.position.x = 0
        #link_msg.pose.position.y = 0
        #link_msg.pose.position.z = 0
        link_msg.pose.orientation.x = ori[0]
        link_msg.pose.orientation.y = ori[1]
        link_msg.pose.orientation.z = ori[2]
        link_msg.pose.orientation.w = ori[3]
        #link_msg.twist.linear.x = 0
        #link_msg.twist.linear.y = 0
        #link_msg.twist.linear.z = 0
        #link_msg.twist.angular.x = 0
        #link_msg.twist.angular.y = 0
        #link_msg.twist.angular.z = 0
        link_msg.reference_frame = 'sdu_master::landing::landing_gear_ref'
        rospy.wait_for_service('/gazebo/set_link_state')
        resp = self.set_link(link_msg)
        #time.sleep(1)
        

    def quaternion_from_euler(self, roll, pitch, yaw): #Angle in radians
        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        
        return [qx, qy, qz, qw]

    def distanceToTarget(self,targetPosition):
        x = targetPosition.pose.position.x - self.current_position.pose.position.x
        y = targetPosition.pose.position.y - self.current_position.pose.position.y
        z = targetPosition.pose.position.z - self.current_position.pose.position.z
        return math.sqrt(x*x + y*y + z*z)

    def fly_to_ship(self):
        header = Header()
        self.old_time = time.time()
        while (True):
            header.stamp = rospy.Time.now()
            if self.landing_allowed:    
                target_pos = self.follow_ship()
                target_pos.header = header    
                #print("Target:", target_pos.pose.position.z)
                #self.target_pos_pub.publish(target_pos)
                self.target_pos_pub2.publish(target_pos)
                
            else:
                target_pos = self.set_target()
                target_pos.header = header
                self.target_pos_pub.publish(target_pos)
                #print(self.distanceToTarget(target_pos))
                #if self.gazebo:
                    #self.reset_link(0,0)
                #self.landing_gear_move(1, 0, 0)
                if(self.distanceToTarget(target_pos) < self.dist_to_target_thresh):
                    if not self.gazebo:
                        self.cv_image = self.cam.get_img()
                        success, dist_aruco, yaw = self.aru.calc_euler(self.cv_image)
                    self.landing_allowed = True
                    

            self.rate.sleep()

if __name__ == '__main__':
    rospy.init_node('drone_control', anonymous=True)
    drone = Drone()
    drone.setup()
    drone.fly_to_ship()
    drone.shutdown_drone()
    rospy.spin()
