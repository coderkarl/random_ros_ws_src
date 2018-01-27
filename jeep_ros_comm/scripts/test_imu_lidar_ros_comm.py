#!/usr/bin/env python

import rospy, math
from geometry_msgs.msg import Twist, Quaternion, Point, Pose, Vector3
from nav_msgs.msg import Odometry
from std_msgs.msg import Int16
import tf

import serial
import sys
import numpy as np
from lidar_map_classes_simple import *

class Arduino2():

    def __init__(self):
        #self.lock = lock
        self.serial = serial.Serial('/dev/imu_arduino', baudrate=115200, timeout=2)
        
    def safe_write(self, val_str):
        #self.lock.acquire()
        self.serial.write(val_str)
        #self.lock.release()

    def safe_read(self):
        #self.lock.acquire()
        val_read_str = self.serial.readline()
        #self.lock.release()
        return val_read_str

class Jeep():
    
    def __init__(self):
        
        rospy.init_node('jeep_comm')
        
        self.odom_pub = rospy.Publisher('odom', Odometry, queue_size=5)
        self.odom_broadcaster = tf.TransformBroadcaster()
        
        self.ardIMU = Arduino2()
        self.bot = MyBot(0,0,0,1,0)
        self.prev_time = rospy.Time.now()
        self.time_sum = 0
        self.theta_sum = 0
        self.vx = 0
        self.roll_rad = 0
        self.pitch_rad = 0
        
    def update_odom(self):
        t2 = rospy.Time.now()
        t1 = self.prev_time
        dt = (t2-t1).to_sec()
        
        gyro_thresh = 0.01 #0.01
        g_bias = 0.016 #0.007
        MAX_DTHETA_GYRO = 5
            
        try:
            self.ardIMU.safe_write('A5/1/')
            accx = self.ardIMU.safe_read()
            accy = self.ardIMU.safe_read()
            accz = self.ardIMU.safe_read()
            self.ardIMU.safe_write('A5/2/')
            gyrox = self.ardIMU.safe_read()
            gyroy = self.ardIMU.safe_read()
            gyroz = self.ardIMU.safe_read()
            #c = 0
            #print("gyro z:")
            #print(gyroz)
            #while(gyroz == '' and c < 10):
            #    c = c+1
            #    gyroz = ardIMU.safe_read()
            #    print('try gyro again')
            #print('raw gyro read: ')
            #print(gyroz)
            accx = float(int(accx)-3000)/100.0
            accy = float(int(accy)-3000)/100.0
            accz = float(int(accz)-3000)/100.0
            gyrox = float(int(gyroz)-1000)/100.0
            gyroy = float(int(gyroz)-1000)/100.0
            gyroz = float(int(gyroz)-1000)/100.0
            #print('gyro rad: ')
            #print(gyroz)
        except:
            accx = 0
            accy = 0
            accz = 9.81
            gyrox = 0
            gyroy = 0
            gyroz = 0
            print 'IMU error'
            print "Unexpected Error:", sys.exc_info()[0]
        finally:
            a=0
        
        # Update odom        
        dmeters = 0
        
        if(abs(gyroz+g_bias) < gyro_thresh):
            gz_dps = 0
            dtheta_gyro_deg = 0
        else:
            gz_dps = (gyroz+g_bias)*180/3.14
            #print('gyro deg/sec')
            #print(gz_dps)
            #if(gz_dps > 0):
            #    gz_dps = gz_dps * 1.2
            #else:
            #    gz_dps = gz_dps * 1.1

            dtheta_gyro_deg = gz_dps*dt

        if(abs(dtheta_gyro_deg) > MAX_DTHETA_GYRO):
            #print 'no gyro'
            dtheta_deg = 0
            use_gyro_flag = False
        else:
            #print 'use gyro'
            dtheta_deg = dtheta_gyro_deg
            use_gyro_flag = True

        #update bot position
        self.bot.move(dmeters,dtheta_deg,use_gyro_flag)
        self.bot.servo_deg = 0
        
        # update bot linear x velocity every 150 msec
        # need to use an np array, then push and pop, moving average
        self.theta_sum = self.theta_sum + dtheta_gyro_deg
        self.time_sum = self.time_sum + dt
        if(self.time_sum > 1):
            print 'delta theta: '
            print self.theta_sum
            print('roll rad: ',self.roll_rad, ', pitch rad: ',self.pitch_rad)
            self.theta_sum = 0
            self.time_sum = 0
        
        #bot.botx*100,bot.boty*100,bot.bot_deg
        odom_quat = tf.transformations.quaternion_from_euler(0, 0, self.bot.bot_deg*3.14/180.0)
        self.odom_broadcaster.sendTransform(
        (self.bot.botx, self.bot.boty, 0.),
        odom_quat,
        t2,
        "base_link",
        "odom"
        )
        
        odom = Odometry()
        odom.header.stamp = t2
        odom.header.frame_id = "odom"

        # set the position
        odom.pose.pose = Pose(Point(self.bot.botx, self.bot.boty, 0.), Quaternion(*odom_quat))

        # set the velocity
        odom.child_frame_id = "base_link"
        odom.twist.twist = Twist(Vector3(self.vx, 0, 0), Vector3(0, 0, gz_dps*3.14/180.0))

        # publish the message
        self.odom_pub.publish(odom)
        
        ##### USE IMU TO PUBLISH TRANSFORM BETWEEN LASER AND BASE
        br = tf.TransformBroadcaster()
        roll_rad = math.asin(accx/9.81) + 0.05
        pitch_rad = math.asin(accy/9.81) -0.055
        self.roll_rad = 0.95*self.roll_rad + 0.05*roll_rad
        self.pitch_rad = 0.95*self.pitch_rad + 0.05*pitch_rad
        laser_quat = tf.transformations.quaternion_from_euler(self.roll_rad, self.pitch_rad, 0)
        br.sendTransform((0,0,0),laser_quat,t2,"laser","base_link")
        #####
        
        self.prev_time = t2
        #loop_time = (rospy.Time.now()-t2).to_sec()
        #print("Loop Time: ");
        #print(loop_time)

if __name__ == '__main__': 
  try:
    jeep = Jeep()
    print("starting jeep test")
    
    #rospy.spin()
    
    r = rospy.Rate(50.0)
    while not rospy.is_shutdown():
        jeep.update_odom()
        r.sleep()
    
  except rospy.ROSInterruptException:
    pass
