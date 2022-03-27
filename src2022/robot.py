#!/usr/bin/env python3
"""
    This is a demo program showing the use of the RobotDrive class,
    specifically it contains the code necessary to operate a robot with
    arcade drive.
"""

import wpilib
from wisdom import SHOOTER, DRIVETRAIN

class MyRobot(wpilib.TimedRobot):

    def robotInit(self):
        """Robot initialization function"""

        self.shooter = SHOOTER()
        self.drivetrain = DRIVETRAIN()
        #self.leds = LEDS()

        # self.camera = wpilib.CameraServer
        # self.camera.launch()

        self.controller = wpilib.PS4Controller(0)

    def autonomousInit(self) -> None:
        return super().autonomousInit()
        # self.leds.setSolidColor("violet")

    def autonomousPeriodic(self) -> None:
        return super().autonomousPeriodic()
        # self.leds.setSolidColor("violet")

    def teleopInit(self):
        """Executed at the start of teleop mode"""
        # self.myRobot.setSafetyEnabled(True)
        return super().teleopInit()

    def teleopPeriodic(self):
        LY = self.controller.getLeftY()
        RY = self.controller.getRightY()
        self.drivetrain.left_drive(LY)
        self.drivetrain.right_drive(RY)

        self.shooter.__set_stage1__(0.35)
        self.shooter.__set_stage2__(-0.75)
        self.shooter.__set_intake__(0.25)
        self.shooter.__set_shooter__(0.70)

        # if self.controller.getShareButton() and self.controller.getOptionsButton():
        #     self.shooter.purge()
        # #     # self.leds.setSolidColor("dark_green")
        
        # if self.controller.getR1Button():
        #     self.shooter.ready_shooter()
        # #     # self.leds.setSolidColor("hot_pink")

        # if self.controller.getR2Button():
        #     self.shooter.advance_ball()
        # #     # self.leds.setSolidColor("red_orange")

        # if self.controller.getCrossButton():
        #     self.shooter.enable_intake()
        # #     # self.leds.setSolidColor("aqua")
        
        # if self.controller.getSquareButton():
        #     self.shooter.disable_intake()
        # #     # self.leds.setSolidColor("gray")
            
        
        # if self.controller.getCircleButton():
        #     self.shooter.disable_stages()
        # #     # self.leds.setSolidColor("dark_blue")

        # if self.controller.getL1Button():
        #     self.drivetrain.shift_up()
        
        # if self.controller.getL2Button():
        #     self.drivetrain.shift_down()
        
        return super().teleopPeriodic()

#%% ###########################################################################

if __name__ == "__main__":
    wpilib.run(MyRobot)