#!/usr/bin/env python

import rospy, math
from geometry_msgs.msg import Twist, Quaternion, Point, Pose, Vector3
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

import tf

import serial
import sys
import numpy as np
from lidar_map_classes_simple import *

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
    
    def __init__(self,ardu):
        
        #rospy.init_node('jeep_comm')
        
        twist_cmd_topic = 'cmd_vel'
        rospy.Subscriber(twist_cmd_topic, Twist, self.drive_callback, queue_size=1)
        
        self.odom_pub = rospy.Publisher('odom', Odometry, queue_size=5)
        self.odom_broadcaster = tf.TransformBroadcaster()
        
        self.ard = ardu #Arduino()
        self.ardIMU = Arduino2()
        self.controller = Controller(self.ard)
        self.speed = 0
        self.steer = 570
        self.bot = MyBot(0,0,0,1,0)
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
            
        print("enc_total: ")
        print(self.enc_total)
        
    def update_odom(self):
        t2 = rospy.Time.now()
        t1 = self.prev_time
        dt = (t2-t1).to_sec()
        
        gyro_thresh = 0.05
        g_bias = 0.007
        MAX_DTHETA_GYRO = 5
            
        try:
            self.ardIMU.safe_write('A5/6/')
            gyroz = self.ardIMU.safe_read()
            #c = 0
            #print("gyro z:")
            #print(gyroz)
            #while(gyroz == '' and c < 10):
            #    c = c+1
            #    gyroz = ardIMU.safe_read()
            #    print('try gyro again')
            gyroz = float(int(gyroz)-1000)/100.0
        except:
            gyroz = 0
            print 'IMU error'
            print "Unexpected Error:", sys.exc_info()[0]
        finally:
            a=0
            
        # Read encoder delta   
        try: 
            self.ard.safe_write('A3/4/')
            s = self.ard.safe_read()
            #print("enc: ")
            #print(s)
            c = 0
            #while(s == '' and c < 10):
            #    c = c +1
            #    s = ardIMU.safe_read()
            #    print('try enc again')
            delta_enc_counts = int(s)
        except:
            delta_enc_counts = 0
            print 'enc error'
            print "Unexpected Error:", sys.exc_info()[0]
        finally:
            a=0
        
        # Update odom
        
        self.enc_total = self.enc_total + delta_enc_counts
        
        dmeters = float(delta_enc_counts)/53.0 #53 counts/meter
        
        if(abs(gyroz) < gyro_thresh):
            gz_dps = 0
            dtheta_gyro_deg = 0
        else:
            gz_dps = (gyroz+g_bias)*180/3.14
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
        self.dist_sum = self.dist_sum + dmeters
        self.time_sum = self.time_sum + dt
        if(self.time_sum > 0.15):
            self.vx = self.dist_sum / self.time_sum
            self.dist_sum = 0
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
        
        self.prev_time = t2
        #loop_time = (rospy.Time.now()-t2).to_sec()
        #print("Loop Time: ");
        #print(loop_time)

if __name__ == '__main__': 
  try:
    print("starting jeep test")
    
    #rospy.spin()
    
    #r = rospy.Rate(50.0)
       
    rospy.init_node('jeep_comm')
    
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
            jeep.update_odom()
            
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
            jeep.update_odom()
            
            ard.safe_write(cmd_servo+str(servo_pos)+'/')
            #rospy.sleep(SERVO_WAIT_MILLIS/1000.0)
            servo_pos = servo_pos - servo_step
            ard.safe_write(lidar_read)
            #rospy.sleep(LIDAR_WAIT_MILLIS/1000.0)
            
            try:    
                s = ard.safe_read()
                lidar_dist = int(s)
            except:
                lidar_dist = 0
                print("Lidar read error")
    
            scan.ranges.append(lidar_dist/100.0)
            #scan.intensities.append(1)  # fake data

        scan_pub.publish(scan)
    
  except rospy.ROSInterruptException:
    pass
