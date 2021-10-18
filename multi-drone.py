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
import threading
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils import uri_helper
from cflib.positioning.motion_commander import MotionCommander

# connection dependencies
from multiprocessing.connection import Listener

# crazyflie identifier
URI1 = uri_helper.uri_from_env(default='radio://0/70/2M/E7E7E7E701')
URI2 = uri_helper.uri_from_env(default='radio://0/70/2M/E7E7E7E702')
logging.basicConfig(level=logging.ERROR)

# obstacle prediction threshold 0.85
THRESHOLD = 0.85 #0.85

#def cf.commander.send_hover_setpoint(vx,vy,yaw, xdistance)
# crazyflie control algorithm 
def cf_controller(URI_def):

    with SyncCrazyflie(URI_def, cf=Crazyflie(rw_cache='./cache')) as scf:
        cf = scf.cf
        cf.param.set_value('kalman.resetEstimation', '1')
        time.sleep(0.1)
        cf.param.set_value('kalman.resetEstimation', '0')
        time.sleep(2)

        # takeoff (0.8 meters) ( =20perhaps decrese the height)
        for y in range(20):
            cf.commander.send_hover_setpoint(0,0,0,y/25) #z =y/25
            time.sleep(0.1)
        
        # move forward (25 sec), hover if obstacle detected #250
        steps = 0
        while(steps <=150): 
            inference = (cf_conn.recv()/5) #prediction values
            
            if (inference > THRESHOLD): #vx,vy,yaw,zdistance 
                cf.commander.send_hover_setpoint(0.0, 0.0, 0.0, 0.8) #right #negative goes left x= 0.05 zdis =0.8
                steps += 1
                time.sleep(0.1) 
                  
            else:
                cf.commander.send_hover_setpoint(0.15, 0.0 , 0.0, 0.8) #x 0.35 z=0.8
                steps += 1
                time.sleep(0.1)
                print(steps)
        

        # land from (0.8 meters) #20
        for y in range(1):
            cf.commander.send_hover_setpoint(0, 0, 0, -y/25) #z= -y/25
            time.sleep(0.1)



#control program for 2nd crazyflie
def cf_controller2(URI_def):
    #print("in")

    with SyncCrazyflie(URI_def, cf=Crazyflie(rw_cache='./cache')) as scf:
        print("in")
        cf = scf.cf
        
        cf.param.set_value('kalman.resetEstimation', '1')
        time.sleep(0.1)
        cf.param.set_value('kalman.resetEstimation', '0')
        time.sleep(2)

         
        '''

        while(steps <= 100):
            inference1 = (cf_conn.recv()/5)

            if(inference1 > THRESHOLD):

                for y in range(20):
                    print("lift off")
                    cf.commander.send_hover_setpoint(0,0,0,y/25)
                    time.sleep(0.1)
                    steps +=1
 
        for y in range(1):
            cf.commander.send_hover_setpoint(0, 0, 0, -y/25) #z= -y/25
            time.sleep(0.1) 
        '''
 
                
        for y in range(20):
                    print("lift off")
                    cf.commander.send_hover_setpoint(0,0,0,y/25)
                    time.sleep(0.1)
                    #steps +=1
        
        steps = 0 
        #move forward for 5 seconds
        while(steps <= 40): 
            print("moving forward")
            cf.commander.send_hover_setpoint(0.15, 0.0, 0.0, 0.8)
            time.sleep(0.1)
            steps +=1
        steps=0
        #hovering
        step=0
        
        while(steps <= 500): 

            inference1 = (cf_conn.recv())
                #print(inference)

            if (inference1 > THRESHOLD): #> =threshold
                cf.commander.send_hover_setpoint(0.0, 0.0, 180, 0.8)
                time.sleep(1)
                cf.commander.send_hover_setpoint(0.0, 0.0, 180, 0.8)
                time.sleep(1)
                
                while(step <=60):
                    print("moving back")
                    #cf.commander.send_hover_setpoint(0.0, 0.0, 180, 0.8)
                    #time.sleep(0.1)
                    cf.commander.send_hover_setpoint(0.15, 0.0, 0.0, 0.8)
                    time.sleep(0.1)
                    step += 1
                    if step == 61:
                        print("landing")
                        for y in range(1):
                                cf.commander.send_hover_setpoint(0, 0, 0, -y/25)
                                time.sleep(0.1)
                        exit()
                        
            else:

                    print("hovering")
                    cf.commander.send_hover_setpoint(0.0, 0.0, 0.0, 0.8)
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
    t1 = threading.Thread(target=cf_controller, args=(URI1,))
    #cf_controller(URI1)
    t2 = threading.Thread(target=cf_controller2, args=(URI2,))
    #cf_controller(URI2)


    # starting thread 1
    t1.start()
    time.sleep(2)
    # starting thread 2
    t2.start()

    # wait until thread 1 is completely executed
    t1.join()
    # wait until thread 2 is completely executed
    t2.join()

    # close connection
    cf_rx.close()
    #cf_rx1.close()