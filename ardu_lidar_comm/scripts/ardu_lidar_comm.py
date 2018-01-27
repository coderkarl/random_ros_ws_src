#!/usr/bin/env python

import rospy, math
from sensor_msgs.msg import LaserScan

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

# $ rosrun tf static_transform_publisher 0.0 0.0 0.0 0.0 0.0 0.0 /base_frame /laser_frame 1000

if __name__ == '__main__':
    rospy.init_node('laser_scan_publisher')
    print("Start ardu lidar comm test")
    scan_pub = rospy.Publisher('scan', LaserScan, queue_size=50)
    
    ard = Arduino()
    print("Connected to arduino")
    lidar_read = 'A3/3/' #using lidar
    cmd_servo = 'A4/2/'
    servo_min = 40
    servo_max = 140
    servo_zero = -83
    servo_hyst = 5
    servo_step = 1
    SERVO_WAIT_MILLIS = 3
    LIDAR_WAIT_MILLIS = 3
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
    scan.range_max = 20.0
    
    
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
                print("Lidar read error")
                lidar_dist = 0
    
            scan.ranges.append(lidar_dist/100.0)
            #scan.intensities.append(1)  # fake data

        scan_pub.publish(scan)

        #r.sleep()
