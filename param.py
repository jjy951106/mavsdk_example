import asyncio
from mavsdk import System

async def param():

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

    # Arming the drone
    print("-- Arming")
    await drone.action.arm()

    # Takeoff the vehicle
    print("-- Taking off")
    await drone.action.takeoff()

    await asyncio.sleep(10)

    await drone.param.set_param_float('MC_ROLLRATE_MAX', 220)

    users = await drone.param.get_param_float('MC_ROLLRATE_MAX')

    await asyncio.sleep(3)

    print(users)

if __name__ == "__main__":

    loop = asyncio.get_event_loop()
    loop.run_until_complete(param())