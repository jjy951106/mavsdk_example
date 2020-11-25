
from drone_key_control import *
import threading
import asyncio
from mavsdk import System
from mavsdk import telemetry
from mavsdk.geofence import Point, Polygon

from dronekit import connect, VehicleMode

"""
async def geofance(drone):
    
    # Fetch the home location coordinates, in order to set a boundary around the home location
    print("Fetching home location coordinates...")
    async for terrain_info in drone.telemetry.home():
        latitude = terrain_info.latitude_deg
        longitude = terrain_info.longitude_deg
        break

    await asyncio.sleep(1)
    
    # Define your geofence boundary
    p1 = Point(latitude - 0.0001, longitude - 0.0001)
    p2 = Point(latitude + 0.0001, longitude - 0.0001)
    p3 = Point(latitude + 0.0001, longitude + 0.0001)
    p4 = Point(latitude - 0.0001, longitude + 0.0001)

    # Create a polygon object using your points
    polygon = Polygon([p1, p2, p3, p4], Polygon.FenceType.INCLUSION)

    # Upload the geofence to your vehicle
    print("Uploading geofence...")
    await drone.geofence.upload_geofence([polygon])

    print("Geofence uploaded!")
    
    await drone.param.set_param_int('GF_ACTION', 2)
    await drone.param.set_param_float('GF_MAX_VER_DIST', 5)
"""
"""
class Dronekit:
    
    vehicle = None
    
    def connect(self):
    
        self.vehicle = connect('127.0.0.1:14030', wait_ready=True) # must use different port

    def mode(self, mode):
    
        self.vehicle.mode = VehicleMode(mode) # ATLCTL MANUAL POSITION LOITER
    
    def print_mode(self):
        
        print("Mode: %s" % self.vehicle.mode.name)
"""        

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
        
    await param(drone)
        
    # Start parallel tasks
    print_position_task = asyncio.ensure_future(print_position(drone))
    print_flight_mode_task = asyncio.ensure_future(print_flight_mode(drone))
    control_task = asyncio.ensure_future(manual_controls(drone))

    running_tasks = [print_position_task, print_flight_mode_task]
    termination_task = asyncio.ensure_future(observe_is_in_air_(drone, running_tasks))
    
    await control_task
    await termination_task

async def manual_controls(drone):
    
    await asyncio.sleep(3)
    
    # 조종 값
    dkc = Drone_keyboard_control(0.5, 0.5, 0.8, 0.2, 1.0)

    # set the manual control input after arming
    await drone.manual_control.set_manual_control_input(
        float(0), float(0), float(0.5), float(0)
    )

    # Arming the drone
    async for is_armed in drone.telemetry.armed():
        if is_armed:
            print(f"-- Already Arming")
        else:
            print(f"-- Arming")
            await drone.action.arm()
        break
    
    dkc.start()
        
    # set the manual control input after arming
    await drone.manual_control.set_manual_control_input(
        float(0), float(0), float(0.5), float(0)
    )

    # start manual control
    print("-- Starting manual control")
    await drone.manual_control.start_position_control()

    while dkc.drone_keyboard:
        await drone.manual_control.set_manual_control_input(dkc.__get__('pitch'), dkc.__get__('roll')
                                                                ,dkc.__get__('throttle'), dkc.__get__('yaw'))
        await asyncio.sleep(0.1)
        
        # level
        if dkc.drone_manual_level == 1:
            await level(drone, 1)
            dkc.drone_manual_level = None
        
        if dkc.drone_manual_level == 2:
            await level(drone, 2)   
            dkc.drone_manual_level = None        
        
        if dkc.drone_manual_level == 3:
            await level(drone, 3)
            dkc.drone_manual_level = None
        
        """
        # mode
        if dkc.drone_mode == "HOLD":
            D.mode("LOITER")
            dkc.drone_mode = None
        
        elif dkc.drone_mode == "POSTION":
            D.mode("POSCTL")
            dkc.drone_mode = None
            
        elif dkc.drone_mode == "ALTHOLD":
            D.mode("ALTCTL")
            dkc.drone_mode = None
            
        elif dkc.drone_mode == "MANUAL":
            D.mode("MANUAL")
            dkc.drone_mode = None
            
        elif dkc.drone_mode == "RTL":
            await drone.action.return_to_launch()
            dkc.drone_mode = None
        """
            
        if dkc.drone_mode == "TAKEOFF":
            await drone.action.takeoff()
            dkc.drone_mode = None

    await asyncio.sleep(1)

    if dkc.drone_space_kill:
        await drone.action.kill()
        
    else:
        try:
            await drone.action.land()
        except:
            print("-- Aleady landing")
            pass
        
async def param(drone):
    
    # LOSS LINK ACTION
    await drone.param.set_param_int('NAV_RCL_ACT', 3)
    
    # TAKEOFF
    await drone.param.set_param_float('MIS_TAKEOFF_ALT', 2) # meter
    
    # RTL
    await drone.param.set_param_float('RTL_DESCEND_ALT', 2)
    await drone.param.set_param_float('RTL_RETURN_ALT', 2)
    
    # GEO
    await drone.param.set_param_int('GF_ACTION', 2)
    await drone.param.set_param_float('GF_MAX_VER_DIST', 0)
    await drone.param.set_param_float('GF_MAX_HOR_DIST', 0)
        
async def level(drone, level):
    
    # MANUAL APPLY
    if level == 1:
        degree = 30
        up = 2
        down = 1
        pitch = 80
        roll = 80
        yaw = 100
        
    if level == 2:
        degree = 40
        up = 16
        down = 8
        pitch = 150
        roll = 150
        yaw = 200
        
    if level == 3:
        degree = 50
        up = 32
        down = 16
        pitch = 220
        roll = 220
        yaw = 400
    
    await drone.param.set_param_float("MPC_MAN_TILT_MAX", degree)
    await drone.param.set_param_float("MPC_Z_VEL_MAX_UP", up)
    await drone.param.set_param_float("MPC_Z_VEL_MAX_DN", down)
    await drone.param.set_param_float("MC_PITCHRATE_MAX", pitch)   
    await drone.param.set_param_float("MC_ROLLRATE_MAX", roll)     
    await drone.param.set_param_float("MC_YAWRATE_MAX", yaw)

# PRINT ONLY ALTITUDE
async def print_altitude(drone):
    """ Prints the altitude when it changes """

    previous_altitude = None

    async for position in drone.telemetry.position():
        altitude = round(position.relative_altitude_m)
        if altitude != previous_altitude:
            previous_altitude = altitude
            print(f"Altitude: {altitude}")

# PRINT FLIGHT MODE
async def print_flight_mode(drone):
    """ Prints the flight mode when it changes """

    previous_flight_mode = None

    async for flight_mode in drone.telemetry.flight_mode():
        if flight_mode is not previous_flight_mode:
            previous_flight_mode = flight_mode
            print(f"Flight mode: {flight_mode}")

# PRINT POSITION ALL        
async def print_position(drone):
    
    previous_latitude_deg = None
    previous_longitude_deg = None
    previous_absolute_altitude_m = None
    previous_relative_altitude_m = None
    
    # CHANGE
    """
    async for position in drone.telemetry.position():
        
        latitude_deg = round(position.latitude_deg, 5)
        longitude_deg = round(position.longitude_deg, 5)
        absolute_altitude_m = round(position.absolute_altitude_m, 1)
        relative_altitude_m = round(position.relative_altitude_m, 1)
        
        if latitude_deg != previous_latitude_deg:
            previous_latitude_deg = latitude_deg
            print("---------------------")
            print(f"latitude_deg : {latitude_deg}")
            
        if longitude_deg != previous_longitude_deg:
            previous_longitude_deg = longitude_deg
            print(f"longitude_deg : {longitude_deg}")
        
        if absolute_altitude_m != previous_absolute_altitude_m:
            previous_absolute_altitude_m = absolute_altitude_m
            print(f"absolute_altitude_m : {absolute_altitude_m}")
            
        if relative_altitude_m != previous_relative_altitude_m:
            previous_relative_altitude_m = relative_altitude_m
            print(f"relative_altitude_m : {relative_altitude_m}")
            print("---------------------")
    """
    
    # TIME        
    while True:
        
        async for position in drone.telemetry.position():
        
            latitude_deg = round(position.latitude_deg, 5)
            longitude_deg = round(position.longitude_deg, 5)
            absolute_altitude_m = round(position.absolute_altitude_m, 1)
            relative_altitude_m = round(position.relative_altitude_m, 1)
            
            break
        
        print("---------------------")
        print(f"latitude_deg : {latitude_deg}")
        print(f"longitude_deg : {longitude_deg}")
        print(f"absolute_altitude_m : {absolute_altitude_m}")
        print(f"relative_altitude_m : {relative_altitude_m}")
        print("---------------------")
        
        await asyncio.sleep(3)

            
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
    
if __name__ == "__main__":
    
    loop = asyncio.get_event_loop()
    loop.run_until_complete(main())
    
    
    
    
    
    
    
    




