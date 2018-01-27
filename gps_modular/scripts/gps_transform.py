#!/usr/bin/env python

import rospy
from math import sin, cos
import numpy as np

from nav_msgs.msg import Odometry
import tf
from geometry_msgs.msg import Twist, Quaternion, Point, Pose, Vector3
from sensor_msgs.msg import NavSatFix, NavSatStatus

import gps
import os

#<dictwrapper: {u'epx': 2.216, u'epy': 2.883, u'epv': 4.772, u'ept': 0.005, u'lon': -89.552955, 
#               u'eps': 5.77, u'epc': 9.54, u'lat': 40.892956667, u'track': 204.7, u'mode': 3,
#               u'time': u'2018-01-13T18:25:10.000Z', u'device': u'/dev/GPS_ultimate', u'climb': 0.0,
#               u'alt': 152.6, u'speed': 0.108, u'class': u'TPV'}>


class GPSManager():
    def __init__(self):
        rospy.init_node('gps_transform')
        
        rospy.Subscriber('fix', NavSatFix, self.gps_callback, queue_size=10)
        self.odom_pub = rospy.Publisher('odom_gps', Odometry, queue_size=5)
        self.odom_broadcaster = tf.TransformBroadcaster()
        self.prev_time = rospy.Time.now()

        self.N0 = 0
        self.E0 = 0
        
        
        self.lat_factor = 0

        self.deg2meters = 111111.11
        self.lat_meters = 0
        self.lon_meters = 0

        self.init_count_limit = 100
        self.init_count = 0
        self.init_count = 0
        self.init_state = True
 
    def gps_callback(self, data):
        t2 = rospy.Time.now()
        t1 = self.prev_time
        self.prev_time = t2
        if(data.status.status >= 0): #GPS FIX
            lon = data.longitude
            lat = data.latitude

            if(self.init_count < self.init_count_limit):
                self.N0 = self.N0 + lat
                print "lat(N0): ", self.N0
                self.E0 = self.E0 + lon
                print "long(E0): ", self.E0
                self.init_count = self.init_count + 1
                if(self.init_count == self.init_count_limit):
                    self.N0 = self.N0/self.init_count_limit
                    print "lat(N0): ", self.N0
                    self.E0 = self.E0/self.init_count_limit
                    print "long(E0): ", self.E0
            else:
                self.init_state = False
                self.lat_factor = cos(lat*3.14159/180.0)
                self.lat_meters = self.deg2meters*(lat-self.N0)
                self.lon_meters = self.deg2meters*self.lat_factor*(lon-self.E0)
                
            #print "lat(N), lon(E): ", self.lat_meters, self.lon_meters
            if(self.init_state):
                print "initializing gps, wait"
            else:
                odom_quat = tf.transformations.quaternion_from_euler(0, 0, 0*3.1416/180.0)
                self.odom_broadcaster.sendTransform(
                (self.lon_meters, self.lat_meters, 0.),
                odom_quat,
                t2,
                "base_link_gps",
                "odom"
                )
                
                odom = Odometry()
                odom.header.stamp = t2
                odom.header.frame_id = "odom"
                # set the position
                odom.pose.pose = Pose(Point(self.lon_meters, self.lat_meters, 0.), Quaternion(*odom_quat))

                # set the velocity
                odom.child_frame_id = "base_link_gps"
                #odom.twist.twist = Twist(Vector3(self.vx, 0, 0), Vector3(0, 0, gz_dps*3.1416/180.0))

                # publish the message
                self.odom_pub.publish(odom)
        else:
            rospy.loginfo("No GPS Fix")
        
        # Raw GPS
        #gps_data = NavSatFix()
        #gps_data.header.stamp = t2
        #gps_data.header.frame_id = "gps_frame"
        #gps_data.status.status = self.mode
        #gps_data.latitude = self.lat
        #gps_data.longitude = self.lon
        #gps_data.position_covariance_type = 2 #Diagonal known
        #gps_data.position_covariance[0] = self.epx**2
        #gps_data.position_covariance[4] = self.epy**2
        #gps_data.position_covariance[8] = self.epv**2
        
        # Publish GPS odometry in meters since start
        

        
if __name__ == '__main__':
    try:
        gps_manager = GPSManager()
        print("Starting GPS Transform")
        rospy.spin()
        #r = rospy.Rate(10.0)
        #while not rospy.is_shutdown():
        #    gps_manager.update_odom()
        #    r.sleep()
            
    except rospy.ROSInterruptException:
        print "ROS Interrup Exception"
        pass
