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

from time import *

form_class = uic.loadUiType("test.ui")[0]

class WindowClass(QMainWindow, form_class) :
        
    drone = System()
    
    s = 1
    t = 1
    l = 1
    a = 1
    
    def __init__(self) :
        super().__init__()
        self.setupUi(self)
        
        loop = asyncio.get_event_loop()
        loop.run_until_complete(self.connect(self.drone))
        #loop.run_until_complete(self.geofence(self.drone, 3))
        
        loop.run_until_complete(self.asy())
        
        #Button_ = asyncio.ensure_future(self.Button(loop))
        
        #await Button_
        
    async def asy(self):
        await asyncio.ensure_future(self.Button())
        
        await asyncio.ensure_future(self.shutdown(self.drone))
        await asyncio.ensure_future(self.takeoff(self.drone, 1.5))
        await asyncio.ensure_future(self.landing(self.drone))
        await asyncio.ensure_future(self.arm(self.drone))
        
        
    async def Button(self):
        
        self.ArmButton.clicked.connect(self.loop_arm)
        self.ShutdownButton.clicked.connect(self.loop_shutdown)
        self.TakeoffButton.clicked.connect(self.loop_takeoff)
        self.LandButton.clicked.connect(self.loop_landing)
        
        """
        self.ForwardButton.clicked.connect(
            lambda :
            loop_VelocityBodyYawspeed(
            loop, self.drone, 0.3, 0.0, 0.0, 0.0))
        
        self.BackwardButton.clicked.connect(
            lambda: loop_VelocityBodyYawspeed(
            loop, self.drone))
        
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
        """
        
    async def geofence(self, drone, Altitude):
        await drone.param.set_param_int('GF_ACTION', 2)
        await drone.param.set_param_float('GF_MAX_VER_DIST', Altitude)
        await asyncio.sleep(5)
        
    def loop_shutdown(self):
        self.s = 0
        
    async def shutdown(self, drone):
        while self.s == 1:
            sleep(3)
            print("1")
        
        await drone.action.kill()
        
    def loop_landing(self):
        self.l = 0
        
    async def landing(self, drone):
        while self.l == 1:
            sleep(3)
            print("2")
        
        print("-- Landing")
        await drone.action.land()
        
    def loop_takeoff(self):
        self.t = 0
    
    async def takeoff(self, drone, meter):
        while self.t == 1:
            sleep(3)
            print("3")
        
        print("-- Takeoff")
        if meter != 0:
            await drone.param.set_param_float('MIS_TAKEOFF_ALT', meter)
            await asyncio.sleep(3)
        await drone.action.takeoff()
        
    def loop_arm(self):
        self.a = 0
    
    async def arm(self, drone):
        while self.a == 1:
            sleep(3)
            print("4")
        
        print(f"-- Arming")
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
    
    myWindow = WindowClass()
    myWindow.show()
    
    app.exec_() 