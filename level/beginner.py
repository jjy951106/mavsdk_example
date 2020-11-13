from mavsdk import System

class Beginner:
    
    """
    This class is for parameter set on Beginner
    """
    
    MC_ROLLRATE_MAX = 50
    MC_PITCHRATE_MAX = 50
    MC_YAWRATE_MAX = 50
    
    def __init__(
        self,
        drone
    ):
        self.drone = drone
        
    async def param_set(
        self
    ):
        await self.drone.param.set_param_float('MC_ROLLRATE_MAX', self.MC_ROLLRATE_MAX)
        await self.drone.param.set_param_float('MC_PITCHRATE_MAX', self.MC_PITCHRATE_MAX)
        await self.drone.param.set_param_float('MC_YAWRATE_MAX', self.MC_YAWRATE_MAX)
    
    """  
    async def param_get(
        self
    ):
        print(f"MC_ROLLRATE_MAX = {await self.drone.param.get_param_float('MC_ROLLRATE_MAX')}")
        print(f"MC_PITCHRATE_MAX = {await self.drone.param.get_param_float('MC_PITCHRATE_MAX')}")
        print(f"MC_YAWRATE_MAX = {await self.drone.param.get_param_float('MC_YAWRATE_MAX')}")
    """