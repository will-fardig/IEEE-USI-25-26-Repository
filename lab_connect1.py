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

import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BCM)

toArduino = 17
fromArduino = 27

GPIO.setup(toArduino, GPIO.OUT)
GPIO.setup(fromArduino, GPIO.IN)

# URI to the Crazyflie to connect to
uri = 'radio://0/80/2M'
logging.basicConfig(level=logging.ERROR)
position_estimate = [0,0]

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

def log_pos_callback(timestamp, data, logconf):
    global position_estimate
    position_estimate[0] = data['stateEstimate.x']
    position_estimate[1] = data['stateEstimate.y']
    print(f"X:{position_estimate[0]}, Y:{position_estimate[1]}")
    time.sleep(1)

if __name__ == '__main__':
    cflib.crtp.init_drivers()

    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        logconf = LogConfig(name="Position", period_in_ms=1000)
        logconf.add_variable('stateEstimate.x', 'float')
        logconf.add_variable('stateEstimate.y', 'float')
        scf.cf.log.add_config(logconf)
        logconf.data_received_cb.add_callback(log_pos_callback)
        
        scf.cf.param.add_update_callback(group="deck", name="bcFlow2",
                                cb=param_deck_flow)
        
        if not deck_attached_event.wait(timeout=3):
            print('No deck detected dumbass')
            sys.exit(1)
                
        while GPIO.input(fromArduino) == 0:
            print("Waiting for Arduino to tell us to do something")
            time.sleep(1)
        scf.cf.platform.send_arming_request(True)
        time.sleep(1)
        
        logconf.start()
        sparkArduino(scf)
        logconf.stop()
        
        GPIO.output(toArduino, GPIO.HIGH)
