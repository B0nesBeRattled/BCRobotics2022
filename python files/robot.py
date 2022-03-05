#!/usr/bin/env python3
"""
    This is a demo program showing the use of the RobotDrive class,
    specifically it contains the code necessary to operate a robot with
    arcade drive.
"""

import wpilib
import ctre
import rev
import numpy as np

directions_dict = {
    0:(1,0), #step_index: (distance in m, heading angle in deg)
    1:(0,90),
    2:(1,0),
    3:(0,90),
}


TOLERANCE = 0.1

class MyRobot(wpilib.TimedRobot):

    def robotInit(self):
        """Robot initialization function"""

        self.wheel_diameter = 0.01
        self.axel_length = 0.1
        self.gear_ratio = 1
        self.encoder_ticks_per_rotation = 4096
        self.left_wheel_position = 0
        self.right_wheel_position = 0
        self.current_step = 0

        self.camera = wpilib.CameraServer
        self.camera.launch()
        self.moved_bool = False
        
        # Start accelerometer using I2C attached to roborio
        # self.accelerometer = wpilib.ADXL345_I2C(wpilib.I2C.Port.kOnboard)

        # object that handles basic drive operations
        self.RightMotor1 = ESCInit(1, pid=True)
        self.RightMotor2 = ESCInit(2)
        self.RightMotor3 = ESCInit(3)
        self.RightMotor2.set(ctre._ctre.TalonSRXControlMode.Follower, 1) # Set as follower for motor 1
        self.RightMotor3.set(ctre._ctre.TalonSRXControlMode.Follower, 1) # Set as follower for motor 1

        self.LeftMotor6 = ESCInit(6, pid=True, inverted=True)
        self.LeftMotor5 = ESCInit(5, inverted=True)
        self.LeftMotor4 = ESCInit(4, inverted=True)
        self.LeftMotor4.set(ctre._ctre.TalonSRXControlMode.Follower, 6) # Set as follower for motor 6
        self.LeftMotor5.set(ctre._ctre.TalonSRXControlMode.Follower, 6) # Set as follower for motor 6

        # joysticks 1 & 2 on the driver station
        self.controller = wpilib.PS4Controller(0)

    def autonomousInit(self) -> None:

        # self.AUTO_rotate(45)
        # self.RightMotor6.set(ctre._ctre.TalonSRXControlMode.MotionMagic, 50)
        # g = self.LeftMotor6.getSelectedSensorPosition(0)
        # self.LeftMotor6.set(ctre._ctre.TalonSRXControlMode.MotionMagic, g)
        
        return super().autonomousInit()

    def autonomousPeriodic(self) -> None:
        # if not self.moved_bool:
        #     self.LeftMotor6.set(ctre._ctre.TalonSRXControlMode.MotionMagic, 10*self.encoder_ticks_per_rotation,
        #                         ctre.DemandType.ArbitraryFeedForward, 0.05)
        #     self.LeftMotor4.follow(self.LeftMotor6) # Set as follower for motor 6
        #     self.LeftMotor5.follow(self.LeftMotor6) # Set as follower for motor 6
        #     self.moved_bool = True
        return super().autonomousPeriodic()

    def teleopInit(self):
        """Executed at the start of teleop mode"""
        # self.myRobot.setSafetyEnabled(True)

    def teleopPeriodic(self):
        self.motor_drive_update()

    def motor_drive_update(self, clampL = 0.25, clampR = 0.25, boost_enable = True):
        """Runs the motors with arcade steering"""

        # Boost on both drive sides is enabled only when both bumpers are pressed simultaniously
        if self.controller.getL1Button() and self.controller.getR1Button() and boost_enable:
            clampL = 1.0
            clampR = 1.0

        Rightval = self.controller.getLeftY() * clampR
        Leftval = self.controller.getRightY() * clampL

        self.set_DriveTrain('right', Rightval)
        self.set_DriveTrain('left', Leftval)    

    def set_DriveTrain(self, side, value, output='percent'):
        if output == 'percent':
            output = ctre._ctre.TalonSRXControlMode.PercentOutput

        elif output == 'position':
            output = ctre._ctre.TalonSRXControlMode.MotionMagic
            value *= -1/self.wheel_diameter*self.gear_ratio*self.encoder_ticks_per_rotation
            
        if side == 'left':
            self.LeftMotor6.set(output, value)    


        elif side == 'right':
            self.RightMotor1.set(output, value)
        
        return None
    

    def AUTO_rotate(self, angle):
        if angle > 180:
            angle = 360 - angle
        rotation_length = self.axel_length/self.wheel_diameter/self.gear_ratio*angle/360*self.encoder_ticks_per_rotation

        self.set_DriveTrain('left', rotation_length, output='position')
        self.set_DriveTrain('right', -rotation_length, output='position')
        return None

    def Enter_The_Matrix(self):

        
        # Get motor position
        current_position_left = self.get_position('left')
        current_position_right = self.get_position('right')

        # Get motor heading
        # heading = self.get_heading()

        # Get current drive step
        (set_position, set_heading) = directions_dict[self.current_step]

        # if sensor position is really really really close to drive step, give new instructions
        if set_position == 0: # moving in place around center axis of robot

            # Calculate expected final positions of left and right wheels
            coeff = self.axel_length/self.wheel_diameter/360.
            final_position_left = self.left_wheel_position + set_heading*coeff
            final_position_right = self.right_wheel_position - set_heading*coeff

            # If wheels are sufficiently close to the final position
            if L2norm(final_position_left, current_position_left, final_position_right, current_position_right) < TOLERANCE and self.current_step+1 < len(directions_dict):
                self.current_step += 1

                (set_position, set_heading) = directions_dict[self.current_step]
                self.left_wheel_position = current_position_left 
                self.right_wheel_position = current_position_right
                self.set_DriveTrain('left', self.left_wheel_position + set_heading*coeff, output='position')
                self.set_DriveTrain('right', self.right_wheel_position - set_heading*coeff, output='position')
                

        # if set_heading == 0: # moving some distance along a straight line
            

        # unpack next step from dictionary

        # else pass

        return None

    def get_position(self, side):
        if side == 'left':
            value = self.LeftMotor6.getSelectedSensorPosition(0)

        elif side == 'right':
            value = self.RightMotor1.getSelectedSensorPosition(0)
        
        value /= -1/self.wheel_diameter*self.gear_ratio*self.encoder_ticks_per_rotation
        return value

    def get_heading(self):
        heading = 0
        return heading

#%% ###########################################################################
def ESCInit(CANAddress, pid=False, inverted=False, kSlotIdx = 0, kPIDLoopIdx=0, kTimeoutMs=50):
    Motor = ctre._ctre.TalonSRX(CANAddress)

    if pid:
        Motor.configFactoryDefault()
        Motor.configSelectedFeedbackSensor(ctre._ctre.TalonSRXFeedbackDevice.CTRE_MagEncoder_Relative, kPIDLoopIdx, kTimeoutMs)
        Motor.setSensorPhase(False)

        # Set relevant frame periods to be at least as fast as periodic rate
        Motor.setStatusFramePeriod(
            ctre._ctre.StatusFrameEnhanced.Status_13_Base_PIDF0, 10, kTimeoutMs
        )
        Motor.setStatusFramePeriod(
            ctre._ctre.StatusFrameEnhanced.Status_10_MotionMagic, 10, kTimeoutMs
        )

        # set the peak and nominal outputs
        Motor.configNominalOutputForward(0, kTimeoutMs)
        Motor.configNominalOutputReverse(0, kTimeoutMs)
        Motor.configPeakOutputForward(0.25, kTimeoutMs)
        Motor.configPeakOutputReverse(-0.25, kTimeoutMs)

        # set closed loop gains in slot0 - see documentation */
        Motor.selectProfileSlot(kSlotIdx, kPIDLoopIdx)
        Motor.config_kF(0, 0, kTimeoutMs)
        Motor.config_kP(0, 0.1, kTimeoutMs)
        # Motor.config_kD(0, 0, kTimeoutMs)
        # Motor.config_kI(0, 0, kTimeoutMs)
        Motor.setNeutralMode(ctre._ctre.NeutralMode.Brake)

        # set acceleration and vcruise velocity - see documentation
        Motor.configMotionCruiseVelocity(15000, kTimeoutMs)
        Motor.configMotionAcceleration(6000, kTimeoutMs)
        Motor.changeMotionControlFramePeriod(25)

        # zero the sensor
        Motor.setSelectedSensorPosition(0, kPIDLoopIdx, kTimeoutMs)

    Motor.setInverted(inverted)
    return Motor    
    
def L2norm(x1, x2, y1, y2):
    return np.sqrt((x1-x2)**2 - (y1-y2)**2)/np.sqrt(x1**2 + y1**2)

#%% ###########################################################################

if __name__ == "__main__":
    wpilib.run(MyRobot)