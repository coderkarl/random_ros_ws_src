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
    
    def __init__(self):
        
        rospy.init_node('jeep_comm')
        
        twist_cmd_topic = 'cmd_vel'
        rospy.Subscriber(twist_cmd_topic, Twist, self.drive_callback, queue_size=1)
        rospy.Subscriber('blade_cmd', Int16, self.blade_callback, queue_size=2)
        
        self.odom_pub = rospy.Publisher('odom', Odometry, queue_size=5)
        self.odom_broadcaster = tf.TransformBroadcaster()
        
        self.ard = Arduino()
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
        self.roll_rad = 0
        self.pitch_rad = 0
        self.blade_status = -1
        
    def drive_callback(self, data):
        linear_scale = 0.02
        wheel_base = 0.76
        max_steer_deg = 18.0
        self.speed = int(data.linear.x / linear_scale)
        if(abs(data.linear.x) > 0):
            curvature = data.angular.z / data.linear.x
            steer_angle_deg = curvature * wheel_base * 180.0/3.14
        else:
            steer_angle_deg = 0
            
        if(data.linear.x == 0 and abs(data.angular.z) > 0): #"turn in place"
            print("Turn in Place: ")
            print(data.angular.z)
            self.speed = int(-1.0 / linear_scale)
            curvature = 1.0/2.5 * np.sign(-data.angular.z)
            steer_angle_deg = curvature * wheel_base * 180.0/3.14
        
        self.steer = 570 - int(-steer_angle_deg * 345.0 / max_steer_deg)
        if self.steer < 0:
            self.steer = 0
        elif self.steer > 1000:
            self.steer = 1000
        if self.speed <> self.controller.speed:
            self.controller.write_speed(self.speed)
        
        if self.steer <> self.controller.steer:
            self.controller.write_steer(self.steer)
            
        print("enc_total: ")
        print(self.enc_total)
        print('roll rad: ',self.roll_rad, ', pitch rad: ',self.pitch_rad)
    
    def blade_callback(self,bld):
        bld_cmd = bld.data
        bld_str = 'A2/8/' + str(bld_cmd) + '/'
        print("pre cmd: ", bld_str)
        if(bld_cmd == 1 and self.blade_status == 0):
            self.ard.safe_write(bld_str)
            self.blade_status = 1
            print("blade cmd: ", bld_str)
        elif(bld_cmd == 0 and self.blade_status != 0):
            self.ard.safe_write(bld_str)
            self.blade_status = 0
            print("blade cmd: ", bld_str)
        
    def update_odom(self):
        t2 = rospy.Time.now()
        t1 = self.prev_time
        dt = (t2-t1).to_sec()
        
        gyro_thresh = 0.01 #0.05
        g_bias = 0.016 #0.007
        MAX_DTHETA_GYRO = 5
            
        try:
            self.ardIMU.safe_write('A5/1/')
            accx = self.ardIMU.safe_read()
            accy = self.ardIMU.safe_read()
            accz = self.ardIMU.safe_read()
            self.ardIMU.safe_write('A5/6/')
            gyroz = self.ardIMU.safe_read()
            #c = 0
            #print("gyro z:")
            #print(gyroz)
            #while(gyroz == '' and c < 10):
            #    c = c+1
            #    gyroz = ardIMU.safe_read()
            #    print('try gyro again')
            accx = float(int(accx)-3000)/100.0
            accy = float(int(accy)-3000)/100.0
            accz = float(int(accz)-3000)/100.0
            gyroz = float(int(gyroz)-1000)/100.0
        except:
            accx = 0
            accy = 0
            accz = 9.81
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
        
        if(abs(gyroz+g_bias) < gyro_thresh):
            gz_dps = 0
            dtheta_gyro_deg = 0
        else:
            gz_dps = (gyroz+g_bias)*180/3.14
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
        
        ##### USE IMU TO PUBLISH TRANSFORM BETWEEN LASER AND BASE
        br = tf.TransformBroadcaster()
        if(abs(accx) < 2 and abs(accy) < 2):
            try:
                roll_rad = math.asin(accx/9.81) + 0.05 + 0.015
                pitch_rad = math.asin(accy/9.81) -0.055 -0.017
            except:
                roll_rad = self.roll_rad
                pitch_rad = self.pitch_rad
                print('asin error for roll or pitch')
        else:
            roll_rad = self.roll_rad
            pitch_rad = self.pitch_rad
            print('accx,y above 2 m/s^2')
                
        self.roll_rad = 0.99*self.roll_rad + 0.01*roll_rad
        self.pitch_rad = 0.99*self.pitch_rad + 0.01*pitch_rad
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
