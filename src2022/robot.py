#!/usr/bin/env python3
"""
    This is a demo program showing the use of the RobotDrive class,
    specifically it contains the code necessary to operate a robot with
    arcade drive.
"""

import wpilib
from wisdom import SHOOTER

class MyRobot(wpilib.TimedRobot):

    def robotInit(self):
        """Robot initialization function"""

        self.shooter = SHOOTER()

        # self.camera = wpilib.CameraServer
        # self.camera.launch()

        # joysticks 1 & 2 on the driver station
        self.controller = wpilib.PS4Controller(0)

    def autonomousInit(self) -> None:
        return super().autonomousInit()

    def autonomousPeriodic(self) -> None:
        return super().autonomousPeriodic()

    def teleopInit(self):
        """Executed at the start of teleop mode"""
        # self.myRobot.setSafetyEnabled(True)
        return super().teleopInit()

    def teleopPeriodic(self):
        if self.controller.getShareButton() and self.controller.getOptionsButton():
            self.shooter.purge()
        
        if self.controller.getR1Button():
            self.shooter.ready_shooter

        if self.controller.getR2Button():
            self.shooter.advance_ball()

        if self.controller.getCrossButton():
            self.shooter.enable_intake()
        
        if self.controller.getSquareButton():
            self.shooter.disable_intake()
        
        if self.controller.getCircleButton():
            self.shooter.disable_stages()

        
        return super().teleopPeriodic()
        
    
    


#%% ###########################################################################

if __name__ == "__main__":
    wpilib.run(MyRobot)