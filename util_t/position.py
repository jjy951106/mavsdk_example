

async def print_position(drone):
    
    async for position in drone.telemetry.position():
        print(position)
        
    await asyncio.ensure_future(print_position(drone))   