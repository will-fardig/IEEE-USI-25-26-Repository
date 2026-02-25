import logging
import sys
import time
from threading import Event

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.positioning.motion_commander import MotionCommander
from cflib.utils import uri_helper

# Handling the in and out from the Pi
# Don't forget to install all of ts
# import RPi.GPIO as GPIO
# GPIO.setmode(GPIO.BCM)

fromArduino = 1 # pin 1, got from Ard
toArduino = 2 # pin 2, sent to Ard

# GPIO.setup(fromArduino, GPIO.IN) # 1 is received from Arduino
# GPIO.setup(toArduino, GPIO.OUT) # 2 is sent to the Arduino

# URI to the Crazyflie to connect to
uri = 'radio://0/80/2M'
logging.basicConfig(level=logging.ERROR)
position_estimate = [0,0,0]

DEFAULT_HEIGHT = 0.5 # this is in meters bruv
deck_attached_event = Event()

def param_deck_flow(_, value_str):
    value = int(value_str)
    if value:
        deck_attached_event.set()
        print('Deck is attached!')
    else:
        print('Deck is NOT attached!')

def sparkArduino(scf):
    with MotionCommander(scf, default_height=DEFAULT_HEIGHT) as mc:
        time.sleep(1)
        mc.right(0.75)
        time.sleep(1)
        mc.left(0.75)
        time.sleep(1)
        mc.stop()
        # GPIO.output(2, GPIO.HIGH)

def log_pos_callback(timestamp, data, logconf):
    global position_estimate
    position_estimate[0] = data['stateEstimate.x']
    position_estimate[1] = data['stateEstimate.y']
    position_estimate[2] = data['pm.batteryLevel']
    print(f"X:{position_estimate[0]}, Y:{position_estimate[1]}, Lowkey waste of a var:{position_estimate[2]:2f}%")
    time.sleep(1)

if __name__ == '__main__':
    cflib.crtp.init_drivers()

    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        logconf = LogConfig(name="Position", period_in_ms=1000)
        logconf.add_variable('stateEstimate.x', 'float')
        logconf.add_variable('stateEstimate.y', 'float')
        logconf.add_variable('pm.batteryLevel', 'float')
        scf.cf.log.add_config(logconf)
        logconf.data_received_cb.add_callback(log_pos_callback)
        
        scf.cf.param.add_update_callback(group="deck", name="bcFlow2",
                                cb=param_deck_flow)
        
        if not deck_attached_event.wait(timeout=3):
            print('No deck detected dumbass')
            sys.exit(1)
        
        # while GPIO.input(1) == 0:
        #     print("Waiting for Arduino to be done")
        #     time.sleep(1)
        scf.cf.platform.send_arming_request(True)
        time.sleep(1)
        
        logconf.start()
        sparkArduino(scf)
        logconf.stop()
