import asyncio
from mavsdk import System

async def run():
    
    drone = System()
    
    # CONNECT
    await drone.connect(system_address="udp://:14540")
    
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"-- Connected to drone with UUID: {state.uuid}")
            break
        
    async for global_lock in drone.telemetry.health():
        if global_lock.is_global_position_ok:
            print("-- Global position state is ok")
            break
        
    # Start parallel tasks
    print_altitude_task = asyncio.ensure_future(print_altitude(drone))
    print_flight_mode_task = asyncio.ensure_future(print_flight_mode(drone))

    running_tasks = [print_altitude_task, print_flight_mode_task]
    termination_task = asyncio.ensure_future(observe_is_in_air_(drone, running_tasks))
    
    # SET PARAM
    await param(drone)
    
    observe_Future = asyncio.ensure_future(observe_is_in_air_(drone, running_tasks))
    
    #ARM
    async for is_armed in drone.telemetry.armed():
        if is_armed:
            print(f"-- Already Arming")
        else:
            print(f"-- Arming")
            await drone.action.arm()
        break
    
    #TAKEOFF
    print(f"-- Takeoff")
    await drone.action.takeoff()
    
    await asyncio.sleep(5)
    
    # GOTO
    """
    flying_alt = absolute_altitude + 20.0
    await drone.action.goto_location(47.399386, 8.535245, flying_alt, 0)
    """
    
    await asyncio.sleep(5)
    
    #RTL
    await drone.action.return_to_launch()
    
    
    # LAND
    """
    print(f"-- Landing")
    await drone.action.land()
    """
    
    await observe_Future

async def param(drone):
    
    # TAKEOFF
    await drone.param.set_param_float('MIS_TAKEOFF_ALT', 2)
    
    # RTL
    await drone.param.set_param_float('RTL_DESCEND_ALT', 2)
    await drone.param.set_param_float('RTL_RETURN_ALT', 2)
    
    # GEO
    await drone.param.set_param_int('GF_ACTION', 2)
    await drone.param.set_param_float('GF_MAX_VER_DIST', 0)
    await drone.param.set_param_float('GF_MAX_HOR_DIST', 0)
    

async def get_relative_altitude_m(drone):
    
    async for position in drone.telemetry.position():
        
        relative_altitude_m = position.relative_altitude_m
    
        break
        
    return relative_altitude_m

async def print_altitude(drone):
    """ Prints the altitude when it changes """

    previous_altitude = None

    async for position in drone.telemetry.position():
        altitude = round(position.relative_altitude_m)
        if altitude != previous_altitude:
            previous_altitude = altitude
            print(f"Altitude: {altitude}")

async def print_flight_mode(drone):
    """ Prints the flight mode when it changes """

    previous_flight_mode = None

    async for flight_mode in drone.telemetry.flight_mode():
        if flight_mode is not previous_flight_mode:
            previous_flight_mode = flight_mode
            print(f"Flight mode: {flight_mode}")

async def observe_is_in_air(drone):

    was_in_air = False

    async for is_in_air in drone.telemetry.in_air():
        if is_in_air:
            was_in_air = is_in_air
            
        if was_in_air and not is_in_air:
            
            print("Detected Landing and Disarming")
            await drone.action.disarm()            
                
            await asyncio.get_event_loop().shutdown_asyncgens()
            
            return  

async def observe_is_in_air_(drone, running_tasks):

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
            
            print("Detected Landing and Disarming")
            await drone.action.disarm()            
                
            await asyncio.get_event_loop().shutdown_asyncgens()
            
            return

if __name__ == "__main__" :
    
    loop = asyncio.get_event_loop()
    
    loop.run_until_complete(run())