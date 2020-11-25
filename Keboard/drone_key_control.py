from pynput import keyboard

import threading
import time

from mavsdk import System

class Drone_keyboard_control(threading.Thread):
    
    drone_keyboard = True
    drone_space_kill = False
    drone_mode = None
    drone_manual_level = None
    
    # get current state of roll axis (between -1 and 1)
    roll = float(0)
    # get current state of pitch axis (between -1 and 1)
    pitch = float(0)
    # get current state of throttle axis (between -1 and 1, but between 0 and 1 is expected)
    throttle = float(0.5)
    # get current state of yaw axis (between -1 and 1)
    yaw = float(0)
    
    roll_Max = float(1)
    pitch_Max = float(1)
    throttle_up_Max = float(1)
    throttle_down_Max = float(0)
    yaw_Max = float(1)
    
    def __init__(
        self,
        roll_Max,
        pitch_Max,
        throttle_up_Max,
        throttle_down_Max,
        yaw_Max
    ):
        threading.Thread.__init__(self)
        self.roll_Max = roll_Max
        self.pitch_Max = pitch_Max
        self.throttle_up_Max = throttle_up_Max
        self.throttle_down_Max = throttle_down_Max
        self.yaw_Max = yaw_Max
    
    def run(self):
        with keyboard.Listener( on_press = self.on_press, 
                               on_release = self.on_release) as listener: 
            listener.join()

    def on_press(self, key):
        
        if key == keyboard.Key.left:
            self.roll = -self.roll_Max
        if key == keyboard.Key.right:
            self.roll = self.roll_Max
        if key == keyboard.Key.up:
            self.pitch = self.pitch_Max
        if key == keyboard.Key.down:
            self.pitch = -self.pitch_Max

        if key == keyboard.KeyCode.from_char('w'):
            self.throttle = self.throttle_up_Max
        if key == keyboard.KeyCode.from_char('s'):
            self.throttle = self.throttle_down_Max
        if key == keyboard.KeyCode.from_char('a'):
            self.yaw = -self.yaw_Max
        if key == keyboard.KeyCode.from_char('d'):
            self.yaw = self.yaw_Max
        
    def on_release(self, key):
        
        if key == keyboard.Key.left:
            self.roll = float(0)
        if key == keyboard.Key.right:
            self.roll = float(0)
        if key == keyboard.Key.up:
            self.pitch = float(0)
        if key == keyboard.Key.down:
            self.pitch = float(0)

        if key == keyboard.KeyCode.from_char('w'):
            self.throttle = float(0.5)
        if key == keyboard.KeyCode.from_char('s'):
            self.throttle = float(0.5)
        if key == keyboard.KeyCode.from_char('a'):
            self.yaw = float(0)
        if key == keyboard.KeyCode.from_char('d'):
            self.yaw = float(0)
        
        if key == keyboard.KeyCode.from_char('1'):
            self.roll_Max = float(0.2)
            self.pitch_Max = float(0.2)
            self.throttle_up_Max = float(0.6)
            self.throttle_down_Max = float(0.3)
            self.yaw_Max = float(0.6)
            
        if key == keyboard.KeyCode.from_char('2'):
            self.roll_Max = float(0.4)
            self.pitch_Max = float(0.4)
            self.throttle_up_Max = float(0.7)
            self.throttle_down_Max = float(0.2)
            self.yaw_Max = float(0.7)
        
        if key == keyboard.KeyCode.from_char('3'):
            self.roll_Max = float(0.6)
            self.pitch_Max = float(0.6)
            self.throttle_up_Max = float(0.8)
            self.throttle_down_Max = float(0.2)
            self.yaw_Max = float(0.6)
        
        if key == keyboard.KeyCode.from_char('4'):
            self.roll_Max = float(0.8)
            self.pitch_Max = float(0.8)
            self.throttle_up_Max = float(0.8)
            self.throttle_down_Max = float(0.1)
            self.yaw_Max = float(0.8)
             
        if key == keyboard.KeyCode.from_char('5'):
            self.roll_Max = float(1.0)
            self.pitch_Max = float(1.0)
            self.throttle_up_Max = float(1.0)
            self.throttle_down_Max = float(0.0)
            self.yaw_Max = float(1.0)

        print('Key %s released' %key) 
        if key == keyboard.Key.esc: # esc 키가 입력되면 종료
            self.drone_keyboard = False
            return False
        
        if key == keyboard.Key.space: # space 키가 입력되면 종료
            self.drone_keyboard = False
            self.drone_space_kill = True
            return False
        
        if key == keyboard.KeyCode.from_char('z'):
            self.drone_manual_level = 1
        
        if key == keyboard.KeyCode.from_char('x'):
            self.drone_manual_level = 2
            
        if key == keyboard.KeyCode.from_char('c'):
            self.drone_manual_level = 3
            
        if key == keyboard.KeyCode.from_char('t'):
            self.drone_mode = "TAKEOFF"
        
    def __get__(self, out):
        if out == 'roll':
            return self.roll
        if out == 'pitch':
            return self.pitch
        if out == 'throttle':
            return self.throttle
        if out == 'yaw':
            return self.yaw
        if out is None:
            return self.pitch, self.roll, self.throttle, self.yaw
    
if __name__ == "__main__":

    dkc = drone_key_control()
    
    dkc.start()
    
    while True:   
        print(dkc.__get__(None))

    time.sleep(5)

    dkc.stop()

