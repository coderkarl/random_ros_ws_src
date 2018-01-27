import numpy as np
import random
import cv2

class MyBot():
    def __init__(self,iX,iY,iDeg,iLH,NM):
        self.bot_deg = iDeg
        self.botx = iX
        self.boty = iY
        self.LH = iLH
        self.servo_deg = 0
        self.NM = NM
        self.x_vec = np.zeros(NM)
        self.y_vec = np.zeros(NM)
        self.deg_vec = np.zeros(NM)
        self.servo_vec = np.zeros(NM)
        self.dist_vec = np.zeros(NM)
    def move(self,dmeters,dtheta,use_gyro_flag):
        
        dmeters_r = dmeters#*random.gauss(1,0.5) #20% std. dev.
        if use_gyro_flag:
            self.bot_deg = self.bot_deg + dtheta#+random.gauss(0,1)
        else:
            dtheta_r = dtheta# + random.gauss(0,3) #3 deg std. dev.
            self.bot_deg = self.bot_deg + dtheta_r*dmeters_r
            
        dx = dmeters_r*np.cos(self.bot_deg*3.14/180) #+ random.gauss(0,0.01)
        #print 'dx = %f' %(dx)
        dy = dmeters_r*np.sin(self.bot_deg*3.14/180) #+ random.gauss(0,0.01)
        self.botx = self.botx + dx
        self.boty = self.boty + dy
    def update_LH(self,G):
        #if G < 0.5:
        self.LH = self.LH*G
        #else:
        #    self.LH = self.LH*0.3+G*0.7
    def store_vecs(self,dist_cm,ind):
        self.x_vec[ind] = self.botx
        self.y_vec[ind] = self.boty
        self.deg_vec[ind] = self.bot_deg
        self.servo_vec[ind] = self.servo_deg
        self.dist_vec[ind] = dist_cm   

class SonarMap():
    def __init__(self,W,H):
        self.MAP_W = W
        self.MAP_H = H
        self.smap = np.ones((self.MAP_H,self.MAP_W,1),np.uint8)*0.2
        self.DIST_LIMIT_CM = 900
        
    def local_sonar_simple(self,servo_deg,dist_cm,x0_cm,y0_cm,bot_deg):
        if(dist_cm < self.DIST_LIMIT_CM):
            global_angle = servo_deg+bot_deg
            map_x = self.MAP_W/2+int(x0_cm/20)+int(dist_cm/20*np.cos(global_angle*3.14/180))
            map_y = self.MAP_H/2-int(y0_cm/20)-int(dist_cm/20*np.sin(global_angle*3.14/180))
            if map_x > 2 and map_y > 2 and map_x < self.MAP_W-2 and map_y < self.MAP_H-2:
                self.smap[map_y][map_x]=0.99
        
        b_x = self.MAP_W/2+int(x0_cm/20)
        b_y = self.MAP_H/2-int(y0_cm/20)
        if b_x > 0 and b_y > 0 and b_x < self.MAP_W and b_y < self.MAP_H:
            self.smap[b_y][b_x]=0.0
            cv2.circle(self.smap,(b_x,b_y),2,0.0)

    def local_sonar_simple_rays(self,servo_deg,dist_cm,x0_cm,y0_cm,bot_deg):
        global_angle = servo_deg+bot_deg
        if(dist_cm < self.DIST_LIMIT_CM):
            map_x = self.MAP_W/2+int(x0_cm/20)+int(dist_cm/20*np.cos(global_angle*3.14/180))
            map_y = self.MAP_H/2-int(y0_cm/20)-int(dist_cm/20*np.sin(global_angle*3.14/180))
            if map_x > 2 and map_y > 2 and map_x < self.MAP_W-2 and map_y < self.MAP_H-2:
                self.smap[map_y][map_x]=0.99
                cv2.circle(self.smap,(map_x,map_y),2,0.99)

        #Remove obstacles from map
        n = int(dist_cm/20)
        mx = int(self.DIST_LIMIT_CM/20)
        if n < mx:
            sub_dist_cm = np.linspace(0,dist_cm,n-1)
        elif(n == mx):
            sub_dist_cm = np.linspace(0,dist_cm,n-1)
        else:
            sub_dist_cm = np.linspace(0,self.DIST_LIMIT_CM,mx)

        for k in range(len(sub_dist_cm)):
            map_x = self.MAP_W/2+int(x0_cm/20)+int(sub_dist_cm[k]/20*np.cos(global_angle*3.14/180))
            map_y = self.MAP_H/2-int(y0_cm/20)-int(sub_dist_cm[k]/20*np.sin(global_angle*3.14/180))
            if map_x > 2 and map_y > 2 and map_x < self.MAP_W-2 and map_y < self.MAP_H-2:
                self.smap[map_y][map_x]=0.2

        #Bot path
        b_x = self.MAP_W/2+int(x0_cm/20)
        b_y = self.MAP_H/2-int(y0_cm/20)
        if b_x > 0 and b_y > 0 and b_x < self.MAP_W and b_y < self.MAP_H:
            self.smap[b_y][b_x]=0.0
            cv2.circle(self.smap,(b_x,b_y),2,0.0)
            
    def local_sonar(self,servo_deg,dist_cm,x0_cm,y0_cm,bot_deg):
        
        #define Pvg based on DIST_LIMIT_CM and dist_cm
        n = int(dist_cm/20)
        mx = int(self.DIST_LIMIT_CM/20)
        if n < mx:
            Pvg = np.ones(n+1)*0.1
            #Pvg[0:n-3] = np.ones(n-3)*0.1
            a = np.maximum(0,n-3)
            du = n+1-a
            tmp = np.array([0.2,0.5,0.9,0.5])
            Pvg[a:n+1] = tmp[-du:]
            sub_dist_cm = np.linspace(0,dist_cm,n+1)
        elif(n == mx):
            Pvg = np.ones(n)*0.1
            #Pvg[0:n-3] = np.ones(n-3)*0.1
            a = np.maximum(0,n-3)
            du = n+1-a
            tmp = np.array([0.2,0.5,0.9])
            Pvg[a:n] = tmp[-du:]
            sub_dist_cm = np.linspace(0,dist_cm,n)
        else:
            Pvg = np.ones(mx)*0.1
            a = int(mx/2)
            b = int(mx*3/4)
            Pvg[a:b] = np.ones(b-a)*0.1
            Pvg[b:mx] = np.ones(mx-b)*0.2
            sub_dist_cm = np.linspace(0,self.DIST_LIMIT_CM,mx)

        #Find Gsub (probability vector of obstacle along line in direction of sensor heading)
        #Gsub is the same length as Pvg
        Gsub = np.ones(len(Pvg))
        for k in range(len(Gsub)):
            Gsub[k] = self.get_G(servo_deg,sub_dist_cm[k],x0_cm,y0_cm,bot_deg)
            global_angle = servo_deg+bot_deg
            map_x = self.MAP_W/2+int(x0_cm/20)+int(sub_dist_cm[k]/20*np.cos(global_angle*3.14/180))
            map_y = self.MAP_H/2-int(y0_cm/20)-int(sub_dist_cm[k]/20*np.sin(global_angle*3.14/180))
            if map_x > 2 and map_y > 2 and map_x < self.MAP_W-2 and map_y < self.MAP_H-2:
                w = 0.9
                if Gsub[k] < 1.0:
                    self.smap[map_y][map_x]=np.mean([Gsub[k],Pvg[k]])*1.0
                    if k >= (len(Gsub)-2):
                        
                       self.smap[map_y-1][map_x]=np.mean([Gsub[k],Pvg[k]])*0.6
                       self.smap[map_y+1][map_x]=np.mean([Gsub[k],Pvg[k]])*0.6
                       self.smap[map_y][map_x-1]=np.mean([Gsub[k],Pvg[k]])*0.6
                       self.smap[map_y][map_x+1]=np.mean([Gsub[k],Pvg[k]])*0.6
                       self.smap[map_y-1][map_x-1]=np.mean([Gsub[k],Pvg[k]])*0.5
                       self.smap[map_y-1][map_x+1]=np.mean([Gsub[k],Pvg[k]])*0.5
                       self.smap[map_y+1][map_x-1]=np.mean([Gsub[k],Pvg[k]])*0.5
                       self.smap[map_y+1][map_x+1]=np.mean([Gsub[k],Pvg[k]])*0.5
                       
        dd = 1

        b_x = self.MAP_W/2+int(x0_cm/20)
        b_y = self.MAP_H/2-int(y0_cm/20)
        if b_x > 0 and b_y > 0 and b_x < self.MAP_W and b_y < self.MAP_H:
            self.smap[b_y][b_x]=0.0
            cv2.circle(self.smap,(b_x,b_y),2,0.0)
    def get_G(self,servo_deg,dist_cm,x0_cm,y0_cm,bot_deg):
        if(dist_cm < self.DIST_LIMIT_CM*1.5 and dist_cm > 0):
            global_angle = servo_deg+bot_deg
            map_x = self.MAP_W/2+int(x0_cm/20)+int(dist_cm/20*np.cos(global_angle*3.14/180))
            map_y = self.MAP_H/2-int(y0_cm/20)-int(dist_cm/20*np.sin(global_angle*3.14/180))
            #print 'global angle: %f, dist_cm: %d, sonar x: %d, sonar y: %d' %(global_angle,dist_cm,(map_x-80)*20,(60-map_y)*20)
            if map_x > 0 and map_y > 0 and map_x < self.MAP_W and map_y < self.MAP_H:
                G = self.smap[map_y][map_x]#/100.0
            else:
                G = 0.5
        else:
            G = 0.5
        return G
