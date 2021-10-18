'''--------------------------------------------------------------
 File:    control.py
 Author:  Griffin Bonner      <griffi1@umbc.edu>                           
 Date:    8.3.2021
 Description: controlling process: receives NN-outputs, moves in 
 positive x-direction at specified velocity until NN-output 
 exceeds threshold, hovers in place, continues to move in 
 positive x-direction. 
--------------------------------------------------------------'''

# crazyflie dependencies 
import time
import logging
import cflib.crtp
import math
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils import uri_helper
from cflib.positioning.motion_commander import MotionCommander

start_time = time.time()
# connection dependencies
from multiprocessing.connection import Listener

# crazyflie identifier
URI = uri_helper.uri_from_env(default='radio://0/70/2M/E7E7E7E701')
logging.basicConfig(level=logging.ERROR)

# obstacle prediction threshold 0.85
THRESHOLD = 0.85 #0.9

#def cf.commander.send_hover_setpoint(vx,vy,yaw, xdistance)
# crazyflie control algorithm 
def cf_controller():

    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:
        cf = scf.cf
        cf.param.set_value('kalman.resetEstimation', '1')
        time.sleep(0.1)
        cf.param.set_value('kalman.resetEstimation', '0')
        time.sleep(2)

        # takeoff (0.8 meters) ( =20perhaps decrese the height)
        for y in range(20):
            #cf.commander.send_hover_setpoint(0,0,0, y/25) #z =y/25
            time.sleep(0.1)
        
        # move forward (25 sec), hover if obstacle detected #250
        steps = 0
        while(steps <= 250): 
            inference = (cf_conn.recv() / 5) #prediction values
            #inference_s = (cf_conn1.recv() )
            
            
            if (inference > THRESHOLD): #vx,vy,yaw,zdistance 
                #yawrate = (inference_s * 30)  #* 5 #steering_angle inference_s #18
                #print("angle_pred = %s and yawrate = %s" % (inference_s, yawrate))
                #cf.commander.send_hover_setpoint(0.0, 0.0, 0.0, 0.8) #right #negative goes left x= 0.05 zdis =0.8
                steps += 1
                time.sleep(0.1) #stop execution  and then turn straight 
                  
                #cf.commander.send_hover_setpoint(0.05, 0.0, -yawrate, 0.4)
                #time.sleep(0.1)
                #cf.commander.send_hover_setpoint(0.05, 0.0, 0.0, 0.4)
                #time.sleep(0.1)
            else:
                #print("angle_pred = %s and yawrate = %s" % (inference_s, yawrate))
                #cf.commander.send_hover_setpoint(0.15, 0.0 , 0.0, 0.8) #x 0.35 z=0.8
                steps += 1
                time.sleep(0.1)
        print(steps)

        # land from (0.8 meters) #20
        for y in range(1):
            #cf.commander.send_hover_setpoint(0, 0, 0, -y/25) #z= -y/25
            time.sleep(0.1)


if __name__ == '__main__':

    # initialize low-level drivers 
    cflib.crtp.init_drivers()

    # connect to inference pipeline (stream.py)
    cf_rx = Listener(('localhost', 6000), authkey=b'crazyflie')
    #cf_rx1 = Listener(('localhost', 7000), authkey=b'crazyflie')

    cf_conn = cf_rx.accept()
    #cf_conn1 = cf_rx1.accept()
    print('connection accepted from', cf_rx.last_accepted)
    #print('connection accepted from', cf_rx1.last_accepted)

    # control crazyflie
    cf_controller()

    # close connection
    cf_rx.close()
    #cf_rx1.close()


