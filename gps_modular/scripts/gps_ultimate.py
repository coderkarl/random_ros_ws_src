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
        rospy.init_node('gps_ultimate')
        
        self.gps_pub = rospy.Publisher('raw_gps', NavSatFix, queue_size = 5)
        self.odom_pub = rospy.Publisher('odom_gps', Odometry, queue_size=5)
        self.odom_broadcaster = tf.TransformBroadcaster()
        self.prev_time = rospy.Time.now()
        # Listen on port 2947 (gpsd) of localhost
        os.system("sudo gpsd /dev/GPS_ultimate -F /var/run/gpsd.sock")
        self.session = gps.gps("localhost", "2947")
        self.session.stream(gps.WATCH_ENABLE | gps.WATCH_NEWSTYLE)

        self.N0 = 0
        self.E0 = 0
        self.lat = 40.89
        self.lon = -90
        self.epx = 100
        self.epy = 100
        self.epv = 100
        self.mode = 0
        
        self.lat_factor = cos(self.lat*3.14159/180.0)

        self.deg2meters = 111111.11
        self.lat_meters = 0
        self.lon_meters = 0

        self.init_count_limit = 10
        self.init_lat_count = 0
        self.init_lon_count = 0
 
    def update_odom(self):
        t2 = rospy.Time.now()
        t1 = self.prev_time
        self.prev_time = t2
        
    	report = self.session.next()
        if report['class'] == 'TPV':
            if hasattr(report,'mode'):
                self.mode = report.mode
            if hasattr(report,'epx'):
                self.epx = report.epx
                self.epy = report.epy
                self.epv = report.epv
            if hasattr(report,'lon'):
                self.lon = report.lon
                if(self.init_lon_count < self.init_count_limit):
                    self.E0 = self.E0 + self.lon
                    print "long(E0): ", self.E0
                    self.init_lon_count = self.init_lon_count + 1
                    if(self.init_lon_count == self.init_count_limit):
                        self.E0 = self.E0/self.init_count_limit
                        print "long(E0): ", self.E0
                else:
                    self.lon_meters = self.deg2meters*self.lat_factor*(self.lon-self.E0)
            if hasattr(report,'lat'):
                self.lat = report.lat
                if (self.init_lat_count < self.init_count_limit):
                    self.N0 = self.N0 + self.lat
                    print "lat(N0): ", self.N0
                    self.init_lat_count = self.init_lat_count + 1
                    if(self.init_lat_count == self.init_count_limit):
                        self.N0 = self.N0/self.init_count_limit
                        print "lat(N0): ", self.N0
                else:
                    self.lat_factor = cos(self.lat*3.14159/180.0)
                    self.lat_meters = self.deg2meters*(self.lat-self.N0)
        
        # Publish Raw GPS
        gps_data = NavSatFix()
        gps_data.header.stamp = t2
        gps_data.header.frame_id = "gps_frame"
        gps_data.status.status = self.mode
        gps_data.latitude = self.lat
        gps_data.longitude = self.lon
        gps_data.position_covariance_type = 2 #Diagonal known
        gps_data.position_covariance[0] = self.epx**2
        gps_data.position_covariance[4] = self.epy**2
        gps_data.position_covariance[8] = self.epv**2
        self.gps_pub.publish(gps_data)
        
        # Publish GPS odometry in meters since start
        
        print "lat(N), lon(E): ", self.lat_meters, self.lon_meters
        if(self.lat_meters == 0):
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
        
if __name__ == '__main__':
    try:
        gps_manager = GPSManager()
        print("Starting GPS")

        r = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            gps_manager.update_odom()
            r.sleep()
            
    except rospy.ROSInterruptException:
        print "ROS Interrup Exception"
        pass
    finally:
        try:
            gps_data.session = None
        except:
            print "No gps session to end"
        print "Goodbye"
