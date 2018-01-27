#-------------------------------------------------------------------------------
# Name:        autojeep
# Purpose:     
#
# Authors:      kak, bjh
# refs:
# http://stackoverflow.com/questions/2846653/python-multithreading-for-dummies
# http://www.tutorialspoint.com/python/python_multithreading.htm
# http://pymotw.com/2/threading/
#-------------------------------------------------------------------------------

import threading
import multiprocessing
import time
import xbox_read
import serial
import signal
import sys
import traceback
import os
from datetime import datetime
from datetime import timedelta
#import numpy as np
import gps
#import kcam
import picamera
import cv2
import numpy as np
from lidar_map_classes_simple import *

class KCam(threading.Thread):
    
    def __init__(self):
        #multiprocessing.Process.__init__(self)
        threading.Thread.__init__(self)
        self.daemon = True
        self.exit = 0
        #self.photoWriteFlag = photoWriteFlag
        self.lidar_pic = []
        self.update_lidar = -1
        self.photo_num = 0
        self.pn = 1
        self.cam = picamera.PiCamera()
        self.cam.resolution = (160,120)
        self.cam.start_preview()
        time.sleep(1)
        print('Camera Initialized')
        
    def run(self):
        TREE_MIN = np.array([22, 170, 0],np.uint8)
        TREE_MAX = np.array([61, 255, 255],np.uint8)
        rect_w = 10; #10
        rect_h = 15;
        noise_se_w = 3;
        noise_se_h = 7;
        fill_se_w = 3;
        fill_se_h = 10;
        rect_se = cv2.getStructuringElement(cv2.MORPH_RECT,(rect_w,rect_h))
        noise_se = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(noise_se_w,noise_se_h))
        fill_se = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(fill_se_w,fill_se_h))
        
        try:
            while True:
                # Get Sonar update
                if self.exit == 1:
                    break
                #photoTime = datetime.now()
                #print 'photo attempt'
                best_cnt = np.array([0])
                self.cam.capture('/home/pi/my_jeep/sync_photos/image'+str(self.pn)+'.jpg','jpeg',use_video_port=True)
                #print 'photo done'
                #photo_time_out = millis(time0)
                #self.photo_num.value = self.photo_num.value + 1 #use just this if you check for change in photo_num
                self.photo_num = self.pn
                self.pn += 1

                #self.cam.capture('my_img0.jpg','jpeg',use_video_port=True)
                #print 'photo stream start'
                orig = cv2.imread('/home/pi/my_jeep/sync_photos/image'+str(self.pn-1)+'.jpg')
                try:
                    hsv = cv2.cvtColor(orig,cv2.COLOR_BGR2HSV)
                except:
                    hsv = orig.copy()
                
                orig_copy = orig.copy()
                tree_filt = cv2.inRange(hsv, TREE_MIN, TREE_MAX)
                opening = cv2.morphologyEx(tree_filt,cv2.MORPH_OPEN,noise_se)
                closing = cv2.morphologyEx(opening,cv2.MORPH_CLOSE, fill_se)
                open2 = cv2.morphologyEx(closing,cv2.MORPH_OPEN, rect_se)
                contours, hierarchy = cv2.findContours(open2,cv2.RETR_TREE,\
                                                          cv2.CHAIN_APPROX_SIMPLE)
                # finding contour with maximum area and store it as best_cnt
                max_area = 0
                for cnt in contours:
                    area = cv2.contourArea(cnt)
                    pts = cnt[:,0]
                    x = pts[:,0]
                    y = pts[:,1]
                    cnt_height = max(y)-min(y)
                    cnt_width = max(x)-min(x)
                    #Longest Distance between 2 points/area
                    if area > max_area and cnt_height/cnt_width > 0.5 and cnt_height < 40 and cnt_width < 30:
                        max_area = area
                        best_cnt = cnt

                # finding centroids of best_cnt and draw a circle there
                if(best_cnt.ndim == 3):
                    M = cv2.moments(best_cnt)
                    cx,cy = int(M['m10']/M['m00']), int(M['m01']/M['m00'])
                    cv2.circle(orig_copy,(cx,cy),5,255,-1)
                    #print "cx: %d, cy: %d" % (cx,cy)
                vis1 = np.concatenate((orig_copy, hsv), axis=0)
                #depth = getDepthMap()
                #depth_resized = 3*cv2.resize(depth,(160,120))
                if self.update_lidar == -1:
                    pic4 = open2
                elif self.update_lidar == 1:
                    pic4 = cv2.resize(self.lidar_pic,(160,120))
                    self.update_lidar = 0
                vis2 = np.concatenate((tree_filt,pic4), axis=0)
                #vis2 = np.concatenate((tree_filt,depth_resized), axis=0)
                vis3 = cv2.cvtColor(vis2,cv2.COLOR_GRAY2RGB)
                vis = np.concatenate((vis1,vis3),axis=1)
                cv2.imwrite('/tmp/stream/pic.jpg',vis)

                #cv2.imwrite('/tmp/stream/pic.jpg',orig)
                #print 'photo stream done'
        except:
            print 'in except'
            #print "Values at Exception: x=%d y=%d " % (x,y) 
            traceback.print_exc(file=sys.stdout)
            self.cam.close()
        print 'close cam'
        self.cam.close()
        #cv2.destroyAllWindows()
            
        return

class GpsPoller(threading.Thread):

    def __init__(self):
        threading.Thread.__init__(self) #threading.Thread(self)
        os.system("sudo gpsd /dev/ttyUSB0 -F /var/run/gpsd.sock")
        self.session = gps.gps("localhost","2947")
        self.session.stream(gps.WATCH_ENABLE | gps.WATCH_NEWSTYLE)
        #self.session = gps(mode=WATCH_ENABLE)
        self.current_value = None
        self.lat = 0
        self.lon = 0
        self.gps_time = 0
        self.exit = 0

    def get_current_value(self):
        return self.current_value

    def run(self):
        try:
            print 'GPS started'
            GPS_WAIT_MILLIS = 1000
            gpsTime = datetime.now()
            
            while self.exit == 0:
                self.current_value = self.session.next()
                time.sleep(0.2) # tune this, you might not get values that quickly
                if(millis(gpsTime) > GPS_WAIT_MILLIS):
                    gpsTime = datetime.now()
                    report = self.get_current_value()
                    if(report['class'] == 'TPV'):
                        if hasattr(report,'lon'):
                            self.lon = report.lon
                        else:
                            self.lon = 0
                        if hasattr(report,'lat'):
                            self.lat = report.lat
                        else:
                            self.lat = 0
                        if hasattr(report,'time'):
                            self.gps_time = report.time
                        else:
                            self.gps_time = -1
                    else:
                        self.lon = 0
                        self.lat = 0
                        self.gps_time = -1
                        
            print 'ended GPS loop'
        except StopIteration:
            print 'stop iteration GPS'
            if self.exit == 0:
                pass

def millis(start_time):
   dt = datetime.now() - start_time
   ms = (dt.days * 24 * 60 * 60 + dt.seconds) * 1000 + dt.microseconds / 1000.0
   return int(ms)


PARAM_DEADZONE = 3000

class Arduino():

    def __init__(self):
        #self.lock = lock
        self.serial = serial.Serial('/dev/ttyACM0', baudrate=115200, timeout=2)
        
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
        self.serial = serial.Serial('/dev/ttyUSB0', baudrate=115200, timeout=2)
        
    def safe_write(self, val_str):
        #self.lock.acquire()
        self.serial.write(val_str)
        #self.lock.release()

    def safe_read(self):
        #self.lock.acquire()
        val_read_str = self.serial.readline()
        #self.lock.release()
        return val_read_str

class Xbox(multiprocessing.Process):
    
    def __init__(self,my_speed,my_steer,my_exit, reset_lidar,auto_mode):
        os.system("sudo rmmod xpad")
        multiprocessing.Process.__init__(self)
        self.daemon = True
        self.read_flag = True
        self.speed = 0
        self.steer = 570
        self.my_exit = my_exit
        self.my_speed = my_speed
        self.my_steer = my_steer
        self.reset_lidar = reset_lidar
        self.auto_mode = auto_mode
        #self.auto_cmd_sent = 0
           
    def run(self):
        count = 0
        for event in xbox_read.event_stream(deadzone=PARAM_DEADZONE):
            #time.sleep(0.1)
            if self.read_flag:
                count = 0
                key = event.key
                val = event.value
                if(self.auto_mode.value == 1):
                    #if(self.auto_cmd_sent == 0):
                        #self.my_speed.value = 50
                        #self.my_steer.value = 225
                        #self.auto_cmd_sent = 1
                    if(key == 'X') and (val == 1):
                        self.auto_mode.value = 0
                        self.speed = 0
                        self.my_speed.value = 0
                        self.steer = 570
                        self.my_steer.value = 570
                else:
                    if key == 'Y1':
                        if abs(val) > PARAM_DEADZONE:
                            self.speed = int(val*100/32768)
                            self.my_speed.value = self.speed
                        else:
                            self.speed = 0
                            self.my_speed.value = self.speed
                    elif key == 'X2':
                        if abs(val) > PARAM_DEADZONE:
                            if(abs(self.speed) > 40):
                                self.steer = int(val/100)+550
                            else:
                                self.steer = int(val/100)+550
                            if(self.steer > 850):
                                self.steer = 850
                            elif(self.steer < 225):
                                self.steer = 225
                            self.my_steer.value = self.steer
                        else:
                            self.steer = 570
                            self.my_steer.value = self.steer
                    elif (key == 'back') and (val == 1):
                        # exit for loop, stop thread
                        self.speed = 0
                        self.my_speed.value = 0
                        self.steer = 570
                        self.my_steer.value = 570
                        self.exit_all = True
                        self.my_exit.value = 1
                        break
                    elif(key == 'Y') and (val == 1):
                        self.reset_lidar.value = 1
                    elif(key == 'Y') and (val == 0):
                        self.reset_lidar.value = 0
                    elif(key == 'start') and (val == 1):
                        self.auto_mode.value = 1
                        #self.auto_cmd_sent = 0
            else:
                time.sleep(0.1)
                count = count + 1
                if(count > 5):
                    count = 0
                    self.speed = 0
                    self.my_speed.value = 0
        return


class Sonar(multiprocessing.Process):
    
    def __init__(self, ard, my_sonar_pos, my_sonar_dist, sonarWriteFlag, my_exit, reset_lidar,delta_enc_counts):
        multiprocessing.Process.__init__(self)
        self.daemon = True
        self.ard = ard
        self.state = 0
        self.count = 0
        self.my_sonar_pos = my_sonar_pos
        self.my_sonar_dist = my_sonar_dist
        self.sonarWriteFlag = sonarWriteFlag
        self.my_exit = my_exit
        self.reset_lidar = reset_lidar
        self.delta_enc_counts = delta_enc_counts
        self.local_reset = 0
        
    def run(self):
        sonar_read = 'A3/3/' #using lidar, actual sonar: 'A3/1/1/'
        read_left_bump = 'A3/2/11/'
        read_right_bump = 'A3/2/12/'
        cmd_reset_lidar = 'A2/7/'
        cmd_servo = 'A4/2/'
        #servo_commands = [40,45,50,55,60,65,70,75,80,85,90,95,100,95,90,85,80,75,70,65,60,55,50,45]
        servo_min = 42
        servo_max = 120
        servo_step = 3
        SERVO_WAIT_MILLIS = servo_step*5
        SONAR_WAIT_MILLIS = 20
        servo_pos = servo_min
        self.ard.safe_write(cmd_servo+str(servo_pos)+'/')
        time.sleep(0.1)
        while True:
            # Get Sonar update
            if self.my_exit.value == 1:
                break
            if self.reset_lidar.value == 1 and self.local_reset == 0:
                self.local_reset = 1
                self.ard.safe_write(cmd_reset_lidar + '0/')
            if self.reset_lidar.value == 0 and self.local_reset == 1:
                self.local_reset = 0
                self.ard.safe_write(cmd_reset_lidar + '1/')
                #self.ard.serial = serial.Serial('/dev/ttyACM0', baudrate=115200, timeout=2)
                print 'pin 7 HIGH'
            if(self.local_reset == 0):
                if self.state == 0:
                    #servo_pos = servo_commands[self.count % len(servo_commands)]
                    servo_pos += servo_step
                    if(servo_pos >= servo_max):
                       servo_step = -servo_step
                    elif(servo_pos <= servo_min):
                       servo_step = -servo_step
                       
                    self.my_sonar_pos.value = servo_pos
                    self.ard.safe_write(cmd_servo + str(servo_pos) + '/')
                    #print('servo command: ' + cmd_servo + str(servo_pos) + '/')
                    self.count += 1
                    servo_time = datetime.now()
                    self.state = 1
                elif(self.state == 1):
                    if(millis(servo_time) > SERVO_WAIT_MILLIS):
                        #self.sonarWriteFlag = 0 #consider this with AND main_Flag that is set to zero after write
                        self.ard.safe_write(sonar_read)
                        self.ard.safe_write('A3/4/')
                        #self.ard.safe_write(read_left_bump)
                        #self.ard.safe_write(read_right_bump)
                        #print('sonar request sent')
                        sonar_time = datetime.now()
                        self.state = 2
                elif(self.state == 2):
                    if(millis(sonar_time) > SONAR_WAIT_MILLIS):
                        #print('sonar read')
                        s = self.ard.safe_read()
                        
                        #lb = self.ard.safe_read()
                        #rb = self.ard.safe_read()
                        try:
                            #print s
                            sonar_dist = int(s)
                            self.my_sonar_dist.value = sonar_dist
                            s = self.ard.safe_read()
                            self.delta_enc_counts.value = int(s)
                            self.sonarWriteFlag.value = 1
                        except ValueError, TypeError:
                            print('error')
                            print "Unexpected Error:", sys.exc_info()
                            print sys.exc_traceback.tb_lineno
                        self.state = 0
        return


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

class Jeep(multiprocessing.Process):
    
    def __init__(self,my_speed,my_steer,my_sonar_pos,my_sonar_dist,sonarWriteFlag,my_exit,reset_lidar,delta_enc_counts):
        multiprocessing.Process.__init__(self)
        self.daemon = True
        #self.lock = threading.Lock()
        
        self.ard = Arduino()
        self.controller = Controller(self.ard)
        self.auto_mode = multiprocessing.Value('i',0)
        self.xbox = Xbox(my_speed,my_steer,my_exit,reset_lidar,self.auto_mode)
        #self.kcam = KCam(photo_num,my_exit)
        #self.gps = None # placeholder
        
        self.sonar = Sonar(self.ard,my_sonar_pos,my_sonar_dist,sonarWriteFlag,my_exit,reset_lidar,delta_enc_counts)
        self.my_speed = my_speed
        self.my_steer = my_steer
        self.my_exit = my_exit
        self.sonar_dist = my_sonar_dist
        self.sonar_pos = my_sonar_pos

    def run(self):
        collision = 0
        col_count = 0
        while True:
            #Need local obstacle avoidance here

            #basic path commands from txt will be passed in main
            #if(self.auto_mode.value == 1 and collision == 0):
            #    self.my_steer.value = 400
            #    self.my_speed.value = 40

            #time.sleep(0.1)
            # check Xbox for new values
            # <> equivalent to !=
            if self.my_exit.value == 1:
                break
            if self.my_speed.value <> self.controller.speed:
                self.controller.write_speed(self.my_speed.value)
                #print self.my_speed.value
            if self.my_steer.value <> self.controller.steer:
                self.controller.write_steer(self.my_steer.value)
                #print self.my_steer.value
        return


def main():
    try:    
        NUM_PART = 1
        N_MEAS = 3
        bot = MyBot(0,0,90,1,N_MEAS)
        sonar_map = SonarMap(320,240)
        f_spd = 0.1
        f_str = 0.5
        speed_filt = 0
        steer_filt = 570
        FWD_SPEED_M = 0.4/25 #0.5/25
        REV_SPEED_M = 0.4/25 #0.4/25
        DTHETA_M = -0.095
        DTHETA_B = 52
        SERVO_DEG_M = 1.1
        SERVO_DEG_B = -4
        SERVO_HYST_DEG = 10
        gyro_thresh = 0.05
        g_bias = 0.007
        MAX_DTHETA_GYRO = 5
        #gpsp = GpsPoller()
        #gpsp.start()

        print "Reading test_path.txt"
        #read test_path.txt before starting threads and processes
        pathX,pathY,pathAction = \
            np.loadtxt('test_path.txt',skiprows=0,unpack = True,usecols=(0,1,2))
        print "Done reading test_path.txt"
        
        kcam = KCam()
        kcam.start()
        print('Start main')
        my_speed = multiprocessing.Value('i',0)
        my_steer = multiprocessing.Value('i',570)
        my_exit = multiprocessing.Value('i',0)
        my_sonar_pos = multiprocessing.Value('i',0)
        my_sonar_dist = multiprocessing.Value('i',0)
        sonarWriteFlag = multiprocessing.Value('i',0)
        reset_lidar = multiprocessing.Value('i',0)
        delta_enc_counts = multiprocessing.Value('i',0)
        #photoWriteFlag = multiprocessing.Value('i',0)
        #photo_num = multiprocessing.Value('i',0)
        #speedWriteFlag = multiprocessing.Value('i',0) #only set every 100 msec
        #Actually, just let the sonar update control the speed update
        
        # create global data structure
        jeep = Jeep(my_speed,my_steer,my_sonar_pos,my_sonar_dist,sonarWriteFlag,my_exit,reset_lidar,delta_enc_counts)

        ardIMU = Arduino2()
        
        # start interleaved task calls

        # start Camera
        #jeep.kcam.start()
        # start Xbox thread
        jeep.xbox.start()
        # start Sonar thread
        jeep.sonar.start()
        # start main Jeep thread
        jeep.start() #jeep is a process
        
        myFile = open('peg_output.txt', 'a')
        myFile.write('Time(ms) photo_num Photo_Time speed steer_pos servo_pos sonar_dist lon lat gps_time magx magy magz gyroz delta_enc_counts\n')
        #photo_num = 0
        photo_time_out = 0
        time0 = datetime.now()
        t1 = int(millis(time0))
        servo_pos1 = my_sonar_pos.value
        enc_read_count = 0
        cum_dist_cm = -1.1

        path_k = len(pathX)
        update_path_time = 1
        path_time = t1
        wait_duration = 1000
        
        # put in loop mode until Ctrl+C or Xbox "Back" is triggered
        while my_exit.value == 0:
            if(sonarWriteFlag.value == 1):
                try:
                    ardIMU.safe_write('A5/1/')
                    ardIMU.safe_write('A5/2/')
                    ardIMU.safe_write('A5/3/')
                    ardIMU.safe_write('A5/6/')
                    magx = ardIMU.safe_read()
                    magx = float(int(magx)-1000)/10.0
                    magy = ardIMU.safe_read()
                    magy = float(int(magy)-1000)/10.0
                    magz = ardIMU.safe_read()
                    magz = float(int(magz)-1000)/10.0
                    gyroz = ardIMU.safe_read()
                    gyroz = float(int(gyroz)-1000)/100.0
                except:
                    magx = -1
                    magy = -1
                    magz = -1
                    gyroz = -1
                    print 'IMU error'
                    print "Unexpected Error:", sys.exc_info()[0]
                finally:
                    a=0

                t2 = int(millis(time0))

                #Path Commands
                if(jeep.auto_mode.value == 1):
                    try:
                        if(update_path_time == 1):
                            print "update auto cmd"
                            path_time = int(millis(time0))
                            update_path_time = 0
                            path_k = path_k - 1
                            if(path_k < 0):
                                path_k = 0
                                
                            motion = pathAction[path_k]
                            print "motion: ", motion
                            if(motion < 0):
                                new_speed = -40
                            elif(motion > 0):
                                new_speed = 40
                            else:
                                new_speed = 0

                            if(motion == -2 or motion == 2):
                                #left
                                my_steer.value = int(250)
                                wait_duration = 3000
                            elif(motion == -3 or motion == 3):
                                #right
                                my_steer.value = int(825)
                                wait_duration = 3000
                            else:
                                my_steer.value = int(570)
                                wait_duration = 1500

                            #handle directional shifts
                            if(my_speed.value * new_speed < 0):
                                my_speed.value = int(0)
                                wait_duration = 1000
                            else:
                                my_speed.value = int(new_speed)
                            print "done update auto cmd"
                        else:
                            if(t2-path_time > wait_duration):
                                update_path_time = 1
                    except:
                        print "auto mode error"
                            
                            
                
                servo_pos2 = my_sonar_pos.value
                #need an error catch and clean close here. cum_dist_cm was not defined and I was stuck
                myFile.write('{} {} {} {} {} {} {} {} {} {} {} {} {} {} {}\n'.format(t2,kcam.photo_num,photo_time_out,my_speed.value,my_steer.value,servo_pos2,my_sonar_dist.value,-1,-1,-1,magx,magy,magz,gyroz,delta_enc_counts.value))
                #photo_num.value = 0 #probably best just to let photo_num be sample and hold. photo_num time is when the transition occurs.
                sonarWriteFlag.value = 0
##                if(gpsp.lon <> 0):
##                    gpsp.lon = 0
##                    gpsp.lat = 0
##                    gpsp.gps_time = -1
                sonar_dist = my_sonar_dist.value
                if(sonar_dist == 0):
                    sonar_dist = 4000
##                if(speed_filt > 0):
##                    dmeters = speed_filt*FWD_SPEED_M*float(t2-t1)/1000.0
##                else:
##                    dmeters = speed_filt*REV_SPEED_M*float(t2-t1)/1000.0
                dmeters = float(delta_enc_counts.value*3)/100.0
                   
                dtheta1 = steer_filt*DTHETA_M + DTHETA_B
                if(abs(gyroz) < gyro_thresh):
                    gz_deg = 0
                    dtheta_g = 0
                else:
                    gz_deg = (gyroz+g_bias)*180/3.14
                    dtheta_g = gz_deg*float(t2-t1)/1000.0

                #print 'Step'
                #print 't1 = %d, t2 = %d' %(t1,t2)
                #print 'gz_deg = %f, dtheta_g = %f' %(gz_deg,dtheta_g)

                if(abs(dtheta_g) > MAX_DTHETA_GYRO):
                    #print 'no gyro'
                    dtheta = dtheta1
                    use_gyro_flag = False
                else:
                    #print 'use gyro'
                    dtheta = dtheta_g
                    use_gyro_flag = True

                #update bot position
                bot.move(dmeters,dtheta,use_gyro_flag)
                
                speed_filt = my_speed.value*(1-f_spd)+speed_filt*f_spd
                steer_filt = my_steer.value*(1-f_str)+steer_filt*f_str


                delta_servo = servo_pos2-servo_pos1
                if(delta_servo < 0):
                    servo_deg = -90 + servo_pos2*SERVO_DEG_M+SERVO_DEG_B+SERVO_HYST_DEG
                else:
                    servo_deg = -90 + servo_pos2*SERVO_DEG_M+SERVO_DEG_B

                bot.servo_deg = servo_deg# + random.gauss(0,1.5) #1.5 deg std dev for particle servo angle

                

                #update map
                sonar_map.local_sonar_simple_rays(servo_deg,sonar_dist,bot.botx*100,bot.boty*100,bot.bot_deg)
                try:
                    im = np.array(sonar_map.smap * 255, dtype = np.uint8)
                    #my_pic = cv2.cvtColor(im,cv2.COLOR_GRAY2RGB)
                    kcam.lidar_pic = im
                    kcam.update_lidar = 1
                    #cv2.imwrite('/tmp/stream/pic.jpg',sonar_map.smap*255)
                except:
                    print 'lidar pic error'
                    print "Unexpected Error:", sys.exc_info()[0]
                t1 = t2

        # Xbox "Back" triggered, so make sure driver is stopped        
        if my_exit.value == 1:
            os.system('sudo killall xboxdrv')
            myFile.close()
            #gpsp.exit = 1
            kcam.exit = 1
    except:
        print "Unexpected Error:", sys.exc_info()[0]
    finally:
        if(my_exit.value == 0):
            myFile.close()
            #gpsp.exit = 1
            kcam.exit = 1
            #cam.close()
            #cv2.destroyAllWindows()

    return


if __name__ == '__main__':
   try:
      print('starting')
      os.system("rm sync_photos/*.jpg -f -v")
      os.system("rm peg_output.txt -f -v")
      main()
      
   finally:
      #os.system('sudo killall gpsd')
      print('goodbye!')
    
