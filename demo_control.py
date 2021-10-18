
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
#URI = 'radio://0/70/2M/E7E7E7E701'
#URI2 = 'radio://0/70/2M/E7E7E7E702'
URI1 = uri_helper.uri_from_env(default='radio://0/70/2M/E7E7E7E701')
URI2 = uri_helper.uri_from_env(default='radio://0/70/2M/E7E7E7E702')

logging.basicConfig(level=logging.ERROR)

#threshold = 0


def cf_controller(URI_def):
    print("in")

    with SyncCrazyflie(URI_def, cf=Crazyflie(rw_cache='./cache')) as scf:
        
        cf = scf.cf
        
        cf.param.set_value('kalman.resetEstimation', '1')
        time.sleep(0.1)
        cf.param.set_value('kalman.resetEstimation', '0')
        time.sleep(2)

        for y in range(20):
            print("lift off")
            cf.commander.send_hover_setpoint(0,0,0,y/25)
            time.sleep(0.1)
        
        for y in range(1):
            cf.commander.send_hover_setpoint(0, 0, 0, -y/25)
            time.sleep(0.5)
'''
        steps=0 
        #move forward for 5 seconds
        while(steps <= 5): 
            print("moving forward")
            #cf.commander.send_hover_setpoint(0.15, 0.0, 0.0, 0.8)
            time.sleep(0.1)
            steps +=1
        
        steps=0
        #hovering
        #cf.commander.send_hover_setpoint(0.0, 0.0, 180, 0.8)
        
        while(steps <= 6): 

            inference = (cf_conn.recv())
            #print(inference)

            if (inference == 1): #> =threshold
                while(steps <=5):
                    print("moving back")
                    #cf.commander.send_hover_setpoint(0.0, 0.0, 180, 0.8)
                    time.sleep(0.1)
                    #cf.commander.send_hover_setpoint(0.0, 0.0, 180, 0.8)
                    steps += 1
                    if steps ==6:
                        print("landing")
                        for y in range(20):
                             #cf.commander.send_hover_setpoint(0, 0, 0, -y/25)
                            time.sleep(0.1)
                        exit()
                        
                        with MotionCommander(scf, default_height=DEFAULT_HEIGHT) as mc:
                            mc.turn_left(180)
                            time.sleep(1)
                            mc.forward(0.5)
                            time.sleep(1)
                        
                        
            else:

                print("hovering")
                #cf.commander.send_hover_setpoint(0.0, 0.0, 0.0, 0.8)
                time.sleep(1)
'''
def cf_controller2(URI_def):
    print("in")

    with SyncCrazyflie(URI_def, cf=Crazyflie(rw_cache='./cache')) as scf:
        
        cf = scf.cf
        
        cf.param.set_value('kalman.resetEstimation', '1')
        time.sleep(0.1)
        cf.param.set_value('kalman.resetEstimation', '0')
        time.sleep(2)

        for y in range(20):
            print("lift off")
            cf.commander.send_hover_setpoint(0,0,0,y/25)
            time.sleep(0.1)
        
        for y in range(1):
            cf.commander.send_hover_setpoint(0, 0, 0, -y/25)
            time.sleep(0.5)

        
if __name__ == '__main__':

    # initialize low-level drivers 
    cflib.crtp.init_drivers()

    # connect to inference pipeline (stream.py)
    cf_rx = Listener(('localhost', 6000), authkey=b'crazyflie')

    cf_conn = cf_rx.accept()
    print('connection accepted from', cf_rx.last_accepted)

    # control crazyflie
    t1 = threading.Thread(target=cf_controller, args=(URI1,))
    #cf_controller(URI1)
    t2 = threading.Thread(target=cf_controller2, args=(URI2,))
    #cf_controller(URI2)


    # starting thread 1
    t1.start()
    time.sleep(0.1)
    # starting thread 2
    t2.start()

    # wait until thread 1 is completely executed
    t1.join()
    # wait until thread 2 is completely executed
    t2.join()

    # close connection
    cf_rx.close()
