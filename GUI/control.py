from pynput import keyboard

import threading
import asyncio
from mavsdk import System
from mavsdk import telemetry

# get current state of roll axis (between -1 and 1)
roll = float(0)
# get current state of pitch axis (between -1 and 1)
pitch = float(0)
# get current state of throttle axis (between -1 and 1, but between 0 and 1 is expected)
throttle = float(0.5)
# get current state of yaw axis (between -1 and 1)
yaw = float(0)

# class 안에서는 self 가 필요함
def on_press(key):
    
    global roll, pitch, throttle, yaw
    
    if key == keyboard.Key.left:
        roll = float(-0.7)
    if key == keyboard.Key.right:
        roll = float(0.7)
    if key == keyboard.Key.up:
        pitch = float(0.7)
    if key == keyboard.Key.down:
        pitch = float(-0.7)

    if key == keyboard.KeyCode.from_char('w'):
        throttle = float(0.8)
    if key == keyboard.KeyCode.from_char('s'):
        throttle = float(0.2)
    if key == keyboard.KeyCode.from_char('a'):
        yaw = float(-0.7)
    if key == keyboard.KeyCode.from_char('d'):
        yaw = float(0.7)    

    print('Key %s pressed' % key) 
        
def on_release(key):

    global roll, pitch, throttle, yaw
    
    if key == keyboard.Key.left:
        roll = float(0)
    if key == keyboard.Key.right:
        roll = float(0)
    if key == keyboard.Key.up:
        pitch = float(0)
    if key == keyboard.Key.down:
        pitch = float(0)

    if key == keyboard.KeyCode.from_char('w'):
        throttle = float(0.5)
    if key == keyboard.KeyCode.from_char('s'):
        throttle = float(0.5)
    if key == keyboard.KeyCode.from_char('a'):
        yaw = float(0)
    if key == keyboard.KeyCode.from_char('d'):
        yaw = float(0)  

    print('Key %s released' %key) 
    if key == keyboard.Key.esc: # esc 키가 입력되면 종료 
        return False

def drone_keyboard():
    with keyboard.Listener( on_press=on_press, on_release=on_release) as listener: 
        listener.join()

async def manual_controls():
    """Main function to connect to the drone and input manual controls"""
    # Connect to the Simulation
    drone = System()
    await drone.connect(system_address="udp://:14540")

    # This waits till a mavlink based drone is connected
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"-- Connected to drone with UUID: {state.uuid}")
            break

    # Checking if Global Position Estimate is ok
    async for global_lock in drone.telemetry.health():
        if global_lock.is_global_position_ok:
            print("-- Global position state is ok")
            break

    # set the manual control input after arming
    await drone.manual_control.set_manual_control_input(
        float(0), float(0), float(0.5), float(0)
    )

    # Arming the drone
    print("-- Arming")
    await drone.action.arm()

    # Takeoff the vehicle
    print("-- Taking off")
    await drone.action.takeoff()
    
    await asyncio.sleep(10)

    # set the manual control input after arming
    await drone.manual_control.set_manual_control_input(
        float(0), float(0), float(0.5), float(0)
    )

    # start manual control
    print("-- Starting manual control")
    await drone.manual_control.start_position_control()

    while True:

        await drone.manual_control.set_manual_control_input(pitch, roll, throttle, yaw)

        await asyncio.sleep(0.1)
    
if __name__ == "__main__":

    
    threading.Thread(target = drone_keyboard).start()

    print("Input keyboard")

    loop = asyncio.get_event_loop()
    loop.run_until_complete(manual_controls())



