
from drone_key_control import *
import threading
import asyncio
from mavsdk import System
from mavsdk import telemetry


async def main():
    
    drone = System()
    
    # Connect to the Simulation  
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
        
    control = asyncio.create_task(manual_controls(drone))
    position = asyncio.create_task(print_position(drone))
    
    await control
    await position

async def manual_controls(drone):
        
    await drone.param.set_param_int('NAV_RCL_ACT', 3)
        
    dkc = Drone_keyboard_control(0.8, 0.8, 0.8, 0.2, 0.8)

    # set the manual control input after arming
    await drone.manual_control.set_manual_control_input(
        float(0), float(0), float(0.5), float(0)
    )

    # Arming the drone
    print("-- Arming")
    await drone.action.arm()
        
    # set the manual control input after arming
    await drone.manual_control.set_manual_control_input(
        float(0), float(0), float(0.5), float(0)
    )

    # start manual control
    print("-- Starting manual control")
    await drone.manual_control.start_position_control()
        
    dkc.start()

    while True:

        await drone.manual_control.set_manual_control_input(dkc.__get__('pitch'), dkc.__get__('roll')
                                                                ,dkc.__get__('throttle'), dkc.__get__('yaw'))
        await asyncio.sleep(0.1)
            
    dkc.stop()
        
async def print_position(drone):
    
    previous_latitude_deg = None
    previous_longitude_deg = None
    # previous_absolute_altitude_m = None
    previous_relative_altitude_m = None
    
    async for position in drone.telemetry.position():
        
        latitude_deg = round(position.latitude_deg, 5)
        longitude_deg = round(position.longitude_deg, 5)
        # absolute_altitude_m = round(position.absolute_altitude_m, 1)
        relative_altitude_m = round(position.relative_altitude_m, 1)
        
        if latitude_deg != previous_latitude_deg:
            previous_latitude_deg = latitude_deg
            print(f"latitude_deg : {latitude_deg}")
            
        if longitude_deg != previous_longitude_deg:
            previous_longitude_deg = longitude_deg
            print(f"longitude_deg : {longitude_deg}")
        
        """    
        if absolute_altitude_m != previous_absolute_altitude_m:
            previous_absolute_altitude_m = absolute_altitude_m
            print(f"{absolute_altitude_m}")
        """
            
        if relative_altitude_m != previous_relative_altitude_m:
            previous_relative_altitude_m = relative_altitude_m
            print(f"relative_altitude_m : {relative_altitude_m}")
            
async def observe_is_in_air(drone, running_tasks):
    """ Monitors whether the drone is flying or not and
    returns after landing """

    was_in_air = False

    async for is_in_air in drone.telemetry.in_air():
        if is_in_air:
            was_in_air = is_in_air

        if was_in_air and not is_in_air:
            for task in running_tasks:
                task.cancel()
                try:
                    await task
                except asyncio.CancelledError:
                    pass
            await asyncio.get_event_loop().shutdown_asyncgens()
            return
    
if __name__ == "__main__":
    
    loop = asyncio.get_event_loop()
    loop.run_until_complete(main())
    
    
    
    
    
    
    
    




