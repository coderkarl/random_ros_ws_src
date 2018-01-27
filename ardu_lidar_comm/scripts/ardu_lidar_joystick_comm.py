#!/usr/bin/env python

import rospy, math
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

import serial
import sys
import numpy as np

class Arduino():

    def __init__(self):
        #self.lock = lock
        self.serial = serial.Serial('/dev/jeep_arduino', baudrate=115200, timeout=2)
        
    def safe_write(self, val_str):
        #self.lock.acquire()
        self.serial.write(val_str)
        #self.lock.release()

    def safe_read(self):
        #self.lock.acquire()
        val_read_str = self.serial.readline()
        #self.lock.release()
        return val_read_str
        
class Controller():
    
    def __init__(self, ard):
        self.ard = ard
        self.speed = 0
        self.steer = 570

    def write_speed(self, val):
        steer_set_right = 'A1/2/875/'
        motor_stop = 'A1/1/1/0/'
        self.speed = val
        if val > 0:
            val_str = 'A1/1/1/' + str(val) + '/'
        elif val < 0:
            val_str = 'A1/1/0/' + str(-val) + '/'
        else:
            val_str = motor_stop
        self.ard.safe_write(val_str)
        return
        
    def write_steer(self, val):
        self.steer = val
        val_str = 'A1/2/' + str(val) + '/'
        self.ard.safe_write(val_str)
        return

class Jeep():
    
    def __init__(self, ardu):
        
        #rospy.init_node('jeep_comm')
        
        twist_cmd_topic = 'cmd_vel'
        rospy.Subscriber(twist_cmd_topic, Twist, self.drive_callback, queue_size=1)
        
        self.ard = ardu
        self.controller = Controller(self.ard)
        self.speed = 0
        self.steer = 570
        self.prev_time = rospy.Time.now()
        self.dist_sum = 0
        self.time_sum = 0
        self.vx = 0
        self.enc_total = 0
        
    def drive_callback(self, data):
        self.speed = int(data.linear.x)
        self.steer = 570 - int(data.angular.z)
        if self.speed <> self.controller.speed:
            self.controller.write_speed(self.speed)
        
        if self.steer <> self.controller.steer:
            self.controller.write_steer(self.steer)

# $ rosrun tf static_transform_publisher 0.0 0.0 0.0 0.0 0.0 0.0 /base_frame /laser_frame 1000

if __name__ == '__main__':
    rospy.init_node('laser_scan_publisher')

    scan_pub = rospy.Publisher('scan', LaserScan, queue_size=50)
    
    ard = Arduino()
    jeep = Jeep(ard)
    lidar_read = 'A3/3/' #using lidar
    cmd_servo = 'A4/2/'
    servo_min = 40
    servo_max = 140
    servo_zero = -83
    servo_hyst = 5
    servo_step = 1
    SERVO_WAIT_MILLIS = 1
    LIDAR_WAIT_MILLIS = 1
    servo_pos = servo_min
    ard.safe_write(cmd_servo+str(servo_pos)+'/')
    rospy.sleep(0.5)
    
    # ard.safe_write(lidar_read)
    # wait
    # s = ard.safe_read()
    # lidar_dist = int(s)

    scan = LaserScan()
    scan.header.frame_id = 'laser'
    scan.range_min = 0.10
    scan.range_max = 15.0
    
    
    #r = rospy.Rate(0.5)
    while not rospy.is_shutdown():
        # min to max angle scan
        servo_pos = servo_min+servo_step
        current_time = rospy.Time.now()
        scan.header.stamp = current_time
        scan.angle_min = (servo_pos+servo_zero)*3.14/180.0
        scan.angle_max = (servo_max+servo_zero)*3.14/180.0
        scan.angle_increment = servo_step*3.14/180.0
        scan.time_increment = 0.009 #(SERVO_WAIT_MILLIS + LIDAR_WAIT_MILLIS)/1000.0

        scan.ranges = []
        #scan.intensities = []
        while(servo_pos <= servo_max):
            #t2 = rospy.Time.now()
            ard.safe_write(cmd_servo+str(servo_pos)+'/') #TAKES ABOUT 1 msec
            #dt = int((rospy.Time.now()-t2).to_nsec()/1000.0)
            #print("After servo cmd:")
            #print(dt)
            #rospy.sleep(SERVO_WAIT_MILLIS/1000.0)
            servo_pos = servo_pos + servo_step
            ard.safe_write(lidar_read) #TAKES ABOUT 1 msec
            #rospy.sleep(LIDAR_WAIT_MILLIS/1000.0)
            #dt = int((rospy.Time.now()-t2).to_nsec()/1000.0)
            #print("After lidar request:")
            #print(dt)
            try:
                s = ard.safe_read() # TAKES ABOUT 6 msec
                lidar_dist = int(s)
            except:
                lidar_dist = 0
                print("Lidar read error")
            #dt = int((rospy.Time.now()-t2).to_nsec()/1000.0)
            #print("After lidar read:")
            #print(dt)
    
            scan.ranges.append(lidar_dist/100.0)
            #scan.intensities.append(1)  # fake data
        
        scan_pub.publish(scan)
        #dt = (rospy.Time.now()-current_time).to_sec()
        #print(dt)
        
        # max to min angle scan
        servo_pos = servo_max - servo_step
        current_time = rospy.Time.now()
        scan.header.stamp = current_time
        scan.angle_min = (servo_pos+servo_zero+servo_hyst)*3.14/180.0
        scan.angle_max = (servo_min+servo_zero+servo_hyst)*3.14/180.0
        scan.angle_increment = -servo_step*3.14/180.0
        scan.time_increment = 0.009 #(SERVO_WAIT_MILLIS + LIDAR_WAIT_MILLIS)/1000.0

        scan.ranges = []
        #scan.intensities = []
        while(servo_pos >= servo_min):
            ard.safe_write(cmd_servo+str(servo_pos)+'/')
            rospy.sleep(SERVO_WAIT_MILLIS/1000.0)
            servo_pos = servo_pos - servo_step
            ard.safe_write(lidar_read)
            rospy.sleep(LIDAR_WAIT_MILLIS/1000.0)
            try:    
                s = ard.safe_read()
                lidar_dist = int(s)
            except:
                lidar_dist = 0
                print("Lidar read error")
    
            scan.ranges.append(lidar_dist/100.0)
            #scan.intensities.append(1)  # fake data

        scan_pub.publish(scan)

        #r.sleep()
