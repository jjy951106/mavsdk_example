
from drone_key_control import *
from drone import *
import threading
import asyncio
from mavsdk import System
from mavsdk import telemetry
from mavsdk.telemetry import FlightMode

async def manual_controls():
    
    """Main function to connect to the drone and input manual controls"""
    # Connect to the Simulation
    a = Drone()
    
    await a.drone_connection(14540)
    
    dkc = drone_key_control()

    # This waits till a mavlink based drone is connected
    async for state in a.drone.core.connection_state():
        if state.is_connected:
            print(f"-- Connected to drone with UUID: {state.uuid}")
            break

    # Checking if Global Position Estimate is ok
    async for global_lock in a.drone.telemetry.health():
        if global_lock.is_global_position_ok:
            print("-- Global position state is ok")
            break

    # set the manual control input after arming
    await a.drone.manual_control.set_manual_control_input(
        float(0), float(0), float(0.5), float(0)
    )

    # Arming the drone
    print("-- Arming")
    await a.drone.action.arm()
    
    # set the manual control input after arming
    await a.drone.manual_control.set_manual_control_input(
        float(0), float(0), float(0.5), float(0)
    )
    
    
    await a.drone.param.set_param_int('NAV_RCL_ACT', 3)
    
    # await drone.action.set_return_to_launch_altitude(5)
    
    # print(await drone.action.get_return_to_launch_altitude())
    

    # start manual control
    print("-- Starting manual control")
    await a.drone.manual_control.start_position_control()
    
    dkc.start()

    while True:

        await a.drone.manual_control.set_manual_control_input(dkc.__get__('pitch'), dkc.__get__('roll')
                                                            ,dkc.__get__('throttle'), dkc.__get__('yaw'))
        await asyncio.sleep(0.1)
        
    dkc.stop()
    
if __name__ == "__main__":

    loop = asyncio.get_event_loop()
    loop.run_until_complete(manual_controls())




