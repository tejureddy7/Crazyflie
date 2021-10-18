
import time
import logging
import cflib.crtp
import math
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils import uri_helper
from cflib.positioning.motion_commander import MotionCommander

# connection dependencies 
from multiprocessing.connection import Listener

# crazyflie identifier
#URI = 'radio://0/70/2M/E7E7E7E701'
#URI2 = 'radio://0/70/2M/E7E7E7E702'
URI1 = uri_helper.uri_from_env(default='radio://0/70/2M/E7E7E7E701')
URI2 = uri_helper.uri_from_env(default='radio://0/70/2M/E7E7E7E702')

logging.basicConfig(level=logging.ERROR)

threshold = 0.85


def cf_controller(URI_def):
    #print("in")

    #while True:
       # msg = cf_conn.recv()
       #if msg > 0.5:
           # break

    with SyncCrazyflie(URI_def, cf=Crazyflie(rw_cache='./cache')) as scf:
        
        cf = scf.cf
        
        cf.param.set_value('kalman.resetEstimation', '1')
        time.sleep(0.1)
        cf.param.set_value('kalman.resetEstimation', '0')
        time.sleep(2)
    #inference = (cf_conn.recv())
                #print(inference)

    #if (inference > threshold):
        for y in range(20):
            print("lift off")
            cf.commander.send_hover_setpoint(0,0,0,y/25)
            time.sleep(0.1)

        for y in range(1):
            cf.commander.send_hover_setpoint(0, 0, 0, -y/25)
            time.sleep(0.1)

        #cf.commander.send_hover_setpoint(0.0, 0.0, 180, 0.8)
        '''
        steps = 0 
        #move forward for 5 seconds
        while(steps <= 70): 
            print("moving forward")
            cf.commander.send_hover_setpoint(0.15, 0.0, 0.0, 0.8)
            time.sleep(0.1)
            steps +=1
        steps=0
        #hovering
        step=0
        
        while(steps <= 500): 

            inference = (cf_conn.recv())
                #print(inference)

            if (inference == 1): #> =threshold
                cf.commander.send_hover_setpoint(0.0, 0.0, 180, 0.8)
                time.sleep(1)
                cf.commander.send_hover_setpoint(0.0, 0.0, 180, 0.8)
                time.sleep(1)
                
                while(step <=70):
                    print("moving back")
                    #cf.commander.send_hover_setpoint(0.0, 0.0, 180, 0.8)
                    #time.sleep(0.1)
                    cf.commander.send_hover_setpoint(0.15, 0.0, 0.0, 0.8)
                    time.sleep(0.1)
                    step += 1
                    if step == 71:
                        print("landing")
                        for y in range(1):
                                cf.commander.send_hover_setpoint(0, 0, 0, -y/25)
                                time.sleep(0.1)
                        exit()
                        
            else:

                    print("hovering")
                    cf.commander.send_hover_setpoint(0.0, 0.0, 0.0, 0.8)
                    time.sleep(1)
                    cf.commander.send_hover_setpoint(0.0, 0.0, 0.0, 0.8)
                    time.sleep(1)

    '''
 
        
if __name__ == '__main__':

    # initialize low-level drivers 
    cflib.crtp.init_drivers()

    # connect to inference pipeline (stream.py)
    #cf_rx = Listener(("10.42.0.1", 6000), authkey=b'secret password')

    #cf_conn = cf_rx.accept()
    print('connection accepted from', cf_rx.last_accepted)


    # control crazyflie
    cf_controller(URI2)

    #cf_controller(URI1)

    # close connection
    cf_rx.close()
