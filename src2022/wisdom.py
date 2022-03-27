'''
Producer-consumer multi-threading reference: https://www.bogotobogo.com/python/Multithread/python_multithreading_Synchronization_Producer_Consumer_using_Queue.php
'''

import rev
import ctre
import wpilib
import wpiutil

# %% SHOOTER CLASSESS

class SHOOTER(object):

    def __init__(self):
        self.__initialize_intake__()
        self.__initialize_shooter__()
        self.__initialize_stage1__()
        self.__initialize_stage2__()

    def purge(self):
        self.__set_intake__(-1)
        self.__set_shooter__(-1)
        self.__set_stage1__(-1)
        self.__set_stage2__(-1)   
        return None  

    def advance_ball(self):
        self.__set_stage1__(-0.25)
        self.__set_stage2__(-0.25) 
        return None

    def ready_shooter(self):
        self.__set_shooter__(0.80)
        return None   

    def disable_intake(self):
        self.__set_intake__(0)
        return None

    def enable_intake(self, inverted=False):
        if inverted == True:
            self.__set_intake__(-1)
        else:
            self.__set_intake__(1)
        return None

    def disable_stages(self):
        self.__set_stage1__(0)
        self.__set_stage2__(0)

    def __initialize_intake__(self):
        self.intake = __singleStage__(3)
        return None
    
    def __initialize_shooter__(self):
        self.__shooter__ = __doubleStage__(4, 5)
        self.__shooter__.invert_motor(5)
        self.__shooter__.invert_motor(4)
        return None

    def __initialize_stage1__(self):
        self.__stage1__ = __singleStage__(1)
        return None

    def __initialize_stage2__(self):
        self.__stage2__ = __singleStage__(2)
        return None

    def __set_intake__(self, value):
        self.intake.set_motors(value)
        return None
    
    def __set_shooter__(self, value):
        self.__shooter__.set_motors(value)
        return None

    def __set_stage1__(self, value):
        self.__stage1__.set_motors(value)
        return None

    def __set_stage2__(self, value):
        self.__stage2__.set_motors(value)
        return None

class __doubleStage__(object):
    def __init__(self, CANAddress1, CANAddress2):
        self.CANAddress1 = CANAddress1
        self.CANAddress2 = CANAddress2
        self.initialize_motors(CANAddress1, CANAddress2)
    
    def initialize_motors(self, CANAddress1, CANAddress2):
        self.motor1 = rev.CANSparkMax(CANAddress1, rev.CANSparkMaxLowLevel.MotorType.kBrushless)
        self.motor1.setInverted(False)

        self.motor2 = rev.CANSparkMax(CANAddress2, rev.CANSparkMaxLowLevel.MotorType.kBrushless)
        self.motor2.setInverted(False)

        self.motor2.follow(self.motor1)
        return None

    def invert_motor(self, id):
        if id == self.CANAddress1:
            self.motor1.setInverted(True)

        elif id == self.CANAddress2:
            self.motor2.setInverted(True)

        return None

    def set_motors(self, value):
        self.motor1.set(value)
        return None

class __singleStage__(object):
    def __init__(self, CANAddress1):
        self.CANAddress1 = CANAddress1
        self.initialize_motors(CANAddress1)

    
    def initialize_motors(self, CANAddress1):
        self.motor1 = rev.CANSparkMax(CANAddress1, rev.CANSparkMaxLowLevel.MotorType.kBrushless)
        self.motor1.setInverted(False)
        return None

    def invert_motor(self):
        self.motor1.setInverted(True)
        return None

    def set_motors(self, value):
        self.motor1.set(value)
        return None

# %% ELEVATOR CLASSES

# class ELEVATOR(object):
#     def __init__(self):
#         self.__elevators__ =  __doubleStage__(6,7)
#         return None

#     def extend_elevators(self):
#         self.__elevators__.set_motors(0.1)

#     def retract_elevators(self):
#         self.__elevators__.set_motors(-0.1)

# %% DRIVETRAIN CLASS

class DRIVETRAIN(object):
    def __init__(self, axel_length=1, wheel_diameter=0.15, ticks_per_rotation=4096, gear_ratio_low=1, gear_ratio_high=10):
        self.current_gear = 0
        self.turning_constant = axel_length/wheel_diameter*ticks_per_rotation/360.0
        self.drive_constant = ticks_per_rotation/wheel_diameter
        self.gear_ratio_low = gear_ratio_low
        self.gear_ratio_high = gear_ratio_high
        self.__initialize_drivetrain__()
        self.__initialize_shifters__()
        return None

    def left_drive(self, speed):
        self.__leftDrive__.set(ctre._ctre.TalonFXControlMode.PercentOutput, speed)
        return None

    def right_drive(self, speed):
        self.__rightDrive__.set(ctre._ctre.TalonFXControlMode.PercentOutput, speed)
        return None

    def move(self, distance):
        # Calculate turning constant to compensate for gear ratio
        if self.current_gear == 0:
            distance = self.gear_ratio_low*self.drive_constant*distance

        elif self.current_gear == 1:
            distance = self.gear_ratio_high*self.drive_constant*distance

        else:
            distance = 0

        # Move Distance
        self.__move_distance__(self.__leftDrive__, distance)
        self.__move_distance__(self.__rightDrive__, distance)
        
        return None
    
    def turn(self, angle):
        # Calculate turning constant to compensate for gear ratio
        if self.current_gear == 0:
            distance = self.gear_ratio_low*self.turning_constant*angle

        elif self.current_gear == 1:
            distance = self.gear_ratio_high*self.turning_constant*angle

        else:
            distance = 0

        # Move Distance
        self.__move_distance__(self.__leftDrive__, distance)
        self.__move_distance__(self.__rightDrive__, -distance)
        return None

    def __initialize_drivetrain__(self):
        self.__leftDrive__ = self.__initialize_drive_side__(10, 11, inverted=True)
        self.__rightDrive__ = self.__initialize_drive_side__(12, 13)     
        return None 

    def __initialize_shifters__(self):
        self.__leftGearbox__ = wpilib.Solenoid(9, wpilib.PneumaticsModuleType.REVPH, 0)
        self.__rightGearbox__ = wpilib.Solenoid(9, wpilib.PneumaticsModuleType.REVPH, 15)
        return None

    def __initialize_drive_side__(self, CANAddress1, CANAddress2, inverted=False):
        motor1 = ctre.TalonFX(CANAddress1)
        motor1.setInverted(inverted)

        motor2 = ctre.TalonFX(CANAddress2)
        motor2.setInverted(inverted)

        motor2.follow(motor1)            

        return motor1 

    def __set_drivetrain__(self, side, value):
        side.set(ctre._ctre.TalonFXControlMode.PercentOutput, value)
        return None

    def __move_distance__(self, side, distance):
        side.set(ctre._ctre.TalonFXControlMode.MotionMagic, distance)
        return None

    def shift_up(self):

        if self.current_gear != 1:
            self.__rightGearbox__.set(1)
            self.__leftGearbox__.set(1)

            # Keep track of current gearbox setting
            self.current_gear = 1

        return None

    def shift_down(self):
        
        if self.current_gear != 0:

            self.__rightGearbox__.set(0)
            self.__leftGearbox__.set(0)

            # Keep track of current gearbox setting
            self.current_gear = 0

        return None 

    def __is_finished__(self):
        if self.__rightDrive__.isMotionProfileFinished() and self.__leftDrive__.isMotionProfileFinished():
            return True

        else:
            return False

# %% LEDS CLASS

class LEDS(object):
    def __init__(self, channel):
        self.blinkin = wpilib.PWMMotorController(0)
        return None

    def setSolidColor(self, color):
        if color == "hot_pink":
            self.blinkin.set(0.57)
        elif color == "dark_red":
            self.blinkin.set(0.59)
        elif color == "red":
            self.blinkin.set(0.61)
        elif color == "red_orange":
            self.blinkin.set(0.63)
        elif color == "orange":
            self.blinkin.set(0.65)
        elif color == "gold":
            self.blinkin.set(0.67)
        elif color == "yellow":
            self.blinkin.set(0.69)
        elif color == "lawn_green":
            self.blinkin.set(0.71)
        elif color == "lime":
            self.blinkin.set(0.73)
        elif color == "dark_green":
            self.blinkin.set(0.75)
        elif color == "green":
            self.blinkin.set(0.77)
        elif color == "blue_green":
            self.blinkin.set(0.79)        
        elif color == "aqua":
            self.blinkin.set(0.81)
        elif color == "sky_blue":
            self.blinkin.set(0.83)
        elif color == "dark_blue":
            self.blinkin.set(0.85)
        elif color == "blue":
            self.blinkin.set(0.87)
        elif color == "blue_violet":
            self.blinkin.set(0.89)
        elif color == "violet":
            self.blinkin.set(0.91)
        elif color == "white":
            self.blinkin.set(0.93)
        elif color == "gray":
            self.blinkin.set(0.95)
        elif color == "dark_gray":
            self.blinkin.set(0.97)
        elif color == "black":
            self.blinkin.set(0.99)
        return None
    
    def disable(self):
        self.blinkin.set(1995)
        return None