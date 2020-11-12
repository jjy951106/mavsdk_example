from mavsdk import System

class Drone:
    
    drone = System()
    
    async def drone_connection(
        self,
        port
    ):
        await self.drone.connect(system_address="udp://:" + str(port))
        
    