import asyncio

from mavsdk.offboard import (OffboardError, PositionNedYaw, VelocityBodyYawspeed)

def loop_PositionNedYaw(loop, drone, North, East, Altitude, Degree):
    loop.run_until_complete(PositionNedYaw_Fuction(drone, North, East, Altitude, Degree))
    #asyncio.gather(PositionNedYaw_Fuction(drone, North, East, Altitude, Degree))

def loop_VelocityBodyYawspeed(loop, drone, Forward, Right, Down, Yaw_Deg):
    loop.run_until_complete(VelocityBodyYawspeed_Function(drone, Forward, Right, Down, Yaw_Deg))
    #asyncio.gather(VelocityBodyYawspeed_Function(drone, Forward, Right, Down, Yaw_Deg))
    
async def PositionNedYaw_Fuction(drone, North, East, Altitude, Degree):
        
    print("-- Setting initial setpoint")
    await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, 0.0, 0.0))

    print("-- Starting offboard")
    try:
        await drone.offboard.start()
    except OffboardError as error:
        print(f"Starting offboard mode failed with error code: \
            {error._result.result}")
        print("-- Disarming")
        await drone.action.disarm()
        return
        
    print(f"-- {-Altitude}m Altitude")
    await drone.offboard.set_position_ned(PositionNedYaw(North, East, Altitude, Degree))
    await asyncio.sleep(5)
        
    print("-- Stopping offboard")
    try:
        await drone.offboard.stop()
    except OffboardError as error:
        print(f"Stopping offboard mode failed with error code: \
            {error._result.result}")
        
async def VelocityBodyYawspeed_Function(drone, Forward, Right, Down, Yaw_Deg):
    
    print("-- Setting initial setpoint")
    await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))

    print("-- Starting offboard")
    try:
        
        await drone.offboard.start()
    except OffboardError as error:
        print(f"Starting offboard mode failed with error code: \
            {error._result.result}")
        print("-- Disarming")
        await drone.action.disarm()
        return
        
    await drone.offboard.set_velocity_body(VelocityBodyYawspeed(Forward, Right, Down, Yaw_Deg))
    await asyncio.sleep(3)
        
    print("-- Stopping offboard")
    try:
        await drone.offboard.stop()
    except OffboardError as error:
        print(f"Stopping offboard mode failed with error code: \
            {error._result.result}")