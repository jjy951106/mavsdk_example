import sys
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

from PyQt5.QtCore import *
from PyQt5.QAxContainer import *

from PyQt5 import uic

import asyncio
from mavsdk import System
from mavsdk import telemetry
from mavsdk.geofence import Point, Polygon

from dronekit import connect, VehicleMode

from offboard import *

form_class = uic.loadUiType("test.ui")[0]

class WindowClass(QMainWindow, form_class) :
        
    drone = System()    
    
    def __init__(self) :
        super().__init__()
        self.setupUi(self)
        
        loop = asyncio.get_event_loop()
        loop.run_until_complete(self.connect(self.drone))
        loop.run_until_complete(self.geofence(self.drone, 3))
        
        """
        asyncio.gather(self.connect(self.drone))
        
        asyncio.sleep(5)
        
        asyncio.gather(self.geofence(self.drone, 3))
        """
        
        self.Button(loop)
        
    def Button(self, loop):
        self.ShutdownButton.clicked.connect(lambda :self.loop_shutdown(loop))
        self.ArmButton.clicked.connect(lambda :self.loop_arm(loop))
        self.TakeoffButton.clicked.connect(lambda :self.loop_takeoff(loop))
        self.LandButton.clicked.connect(lambda :self.loop_landing(loop))
        
        self.ForwardButton.clicked.connect(
            lambda: loop_VelocityBodyYawspeed(
            loop, self.drone, 0.3, 0.0, 0.0, 0.0))
        
        self.BackwardButton.clicked.connect(
            lambda: loop_VelocityBodyYawspeed(
            loop, self.drone, -0.3, 0.0, 0.0, 0.0))
        
        self.RightButton.clicked.connect(
            lambda: loop_VelocityBodyYawspeed(
            loop, self.drone, 0.0, 0.3, 0.0, 0.0))
        
        self.LeftButton.clicked.connect(
            lambda: loop_VelocityBodyYawspeed(
            loop, self.drone, 0.0, -0.3, 0.0, 0.0))
        
        self.UpButton.clicked.connect(
            lambda: loop_VelocityBodyYawspeed(
            loop, self.drone, 0.0, 0.0, -0.2, 0.0))
        
        self.DownButton.clicked.connect(
            lambda: loop_VelocityBodyYawspeed(
            loop, self.drone, 0.0, 0.0, 0.2, 0.0))
        
        self.HomeButton.clicked.connect(
            lambda: loop_PositionNedYaw(
            loop, self.drone, 0.0, 0.0, -1.0, 0.0))
        
    async def geofence(self, drone, Altitude):
        await drone.param.set_param_int('GF_ACTION', 2)
        await drone.param.set_param_float('GF_MAX_VER_DIST', Altitude)
        await asyncio.sleep(5)
        
    def loop_shutdown(self, loop):
        loop.run_until_complete(self.shutdown(self.drone))
        #asyncio.gather(self.shutdown(self.drone))
        
    async def shutdown(self, drone):
        await drone.action.kill()
        
    def loop_landing(self, loop):
        loop.run_until_complete(self.landing(self.drone))
        #asyncio.gather(self.landing(self.drone))
        
    async def landing(self, drone):
        print("-- Landing")
        await drone.action.land()
        
    def loop_takeoff(self, loop):
        loop.run_until_complete(self.takeoff(self.drone, 1.5))
        #asyncio.gather(self.takeoff(self.drone, 1.5))
    
    async def takeoff(self, drone, meter):
        print("-- Takeoff")
        if meter != 0:
            await drone.param.set_param_float('MIS_TAKEOFF_ALT', meter)
            await asyncio.sleep(3)
        await drone.action.takeoff()
        
    def loop_arm(self, loop):
        loop.run_until_complete(self.arm(self.drone))
        #asyncio.gather(self.arm(self.drone))
    
    async def arm(self, drone):
        print("-- Arming")
        await drone.action.arm()
        
    async def connect(self, drone):
        
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
            
        
if __name__ == "__main__" :
    
    app = QApplication(sys.argv)
    
    #loop = QEventLoop(app)
    
    myWindow = WindowClass()
    myWindow.show()
    
    app.exec_() 
    
    #with loop:
    #    sys.exit(loop.run_forever())