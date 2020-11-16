from mavsdk import System
from mavsdk.telemetry import *
import asyncio
import threading

class Drone(threading.Thread):
    
    def __init__(
        self,
        drone
    ):
        threading.Thread.__init__(self)
        self.drone = drone
        
    async def drone_connection(
        self,
        port
    ):
        await self.drone.connect(system_address="udp://:" + str(port))
        
    async def print_position(
        self,
        drone
    ):
        previous_position = None
        
        async for position in self.drone.telemetry.position():
            if position is not previous_position:
                previous_position = position
                print(f"{position}")
                
    async def start_position(
        self
    ):
        await asyncio.ensure_future(print_position(self.drone))
            
    def run(
        self
    ):
        loop = asyncio.get_event_loop()
        loop.run_until_complete(start_position())
        
    