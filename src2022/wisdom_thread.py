'''
Producer-consumer multi-threading reference: https://www.bogotobogo.com/python/Multithread/python_multithreading_Synchronization_Producer_Consumer_using_Queue.php
'''

import rev
import ctre
import threading 
import queue
import wpilib
import wpiutil

# %% SHOOTER CLASSESS

class SHOOTER(object):
    def __init__(self):
        self.shooter = shooterThread(name='shooter')
        self.queue = self.shooter.get_queue()
        self.shooter.start()
        return None

    def __send_command__(self, command, option, value):
        self.queue.put((command, option, value))
        return False

    def purge(self):
        self.__send_command__('purge', None, None)
        return None

    def advance_ball(self):
        self.__send_command__('advance ball', None, None)
        return None

    def ready_shooter(self):
        self.__send_command__('ready shooter', None, None)
        return None

    def disable_intake(self):
        self.__send_command__('disable intake', None, None)
        return None
    
    def enable_intake(self, inverted=False):
        self.__send_command__('enable intake', None, inverted)
        return None

    def disable_stages(self):
        self.__send_command__('disable stages', None, None)
        return None

class shooterThread(threading.Thread):

    def __init__(self, group=None, target=None, name=None,
                 args=(), kwargs=None, verbose=None):
        super(shooterThread,self).__init__()
        self.target = target
        self.name = name
        self.__initialize_intake__()
        self.__initialize_shooter__()
        self.__initialize_stage1__()
        self.__initialize_stage2__()
        return None

    def get_queue(self, buffer_size=10):
        self.queue = queue.Queue(buffer_size)
        return self.queue

    def run(self):

        # Start infinite loop
        while True:

            # Take element from queue
            (command, option, value) = self.queue.get()

            if command == 'purge':
                self.__set_intake__(-1)
                self.__set_shooter__(-1)
                self.__set_stage1__(-1)
                self.__set_stage2__(-1)

            elif command == 'advance ball':
                self.__set_stage1__(1)
                self.__set_stage2__(1)

            elif command == 'ready shooter':
                self.__set_shooter__(1)
            
            elif command == 'disable intake':
                self.__set_intake__(0)

            elif command == 'enable intake':
                if value == True:
                    self.__set_intake__(-1)
                else:
                    self.__set_intake__(1)

            elif command == 'disable stages':
                self.__set_stage1__(0)
                self.__set_stage2__(0)

        return None

    def __initialize_intake__(self):
        self.intake = __singleStage__(3)
        return None
    
    def __initialize_shooter__(self):
        self.__shooter__ = __doubleStage__(4, 5)
        self.__shooter__.invert_motor(5)
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

class ELEVATOR(object):
    def __init__(self):
        return None

# %% DRIVETRAIN CLASS

class DRIVETRAIN(object):
    def __init__(self):
        self.drivetrain = drivetrainThread(name='drivetrain')
        self.queue = self.drivetrain.get_queue()
        self.drivetrain.start()
        
        return None

    def __send_command__(self, command, option, value):
        self.queue.put((command, option, value))
        return False

    def left_drive(self, value):
        self.__send_command__('left drive', None, value)
        return None

    def right_drive(self, value):
        self.__send_command__('right drive', None, value)
        return None

    def shift_up(self):
        self.__send_command__('shift up', None, None)
        return None

    def shift_down(self):
        self.__send_command__('shift down', None, None)
        return None
    
    def turn(self, angle):
        self.__send_command__('turn', None, angle)
        return None

class drivetrainThread(threading.Thread):
    def __init__(self, group=None, target=None, name=None,
                 args=(), kwargs=None, verbose=None, 
                 axel_length=1, wheel_diameter=0.15, ticks_per_rotation=4096, gear_ratio_low=1, gear_ratio_high=10):
        super(drivetrainThread,self).__init__()
        self.target = target
        self.name = name
        self.current_gear = wpilib.DoubleSolenoid.Value.kReverse
        self.turning_constant = axel_length/wheel_diameter*ticks_per_rotation/360.0
        self.drive_constant = ticks_per_rotation/wheel_diameter
        self.gear_ratio_low = gear_ratio_low
        self.gear_ratio_high = gear_ratio_high
        self.__initialize_drivetrain__()
        return None

    def __initialize_drivetrain__(self):
        self.__leftDrive__ = self.__initialize_drive_side__(10, 11)
        self.__rightDrive__ = self.__initialize_drive_side__(12, 13, inverted=True)     
        return None 

    def __initialize_shifters__(self):
        self.__leftGearbox__ = wpilib.DoubleSolenoid(0, wpilib.PneumaticsModuleType.REVPH, 0)
        self.__leftGearbox__.set(wpilib.DoubleSolenoid.Value.kOff)
        
        self.__rightGearbox__ = wpilib.DoubleSolenoid(0, wpilib.PneumaticsModuleType.REVPH, 1)
        self.__leftGearbox__.set(wpilib.DoubleSolenoid.Value.kOff)
        return None

    def __initialize_drive_side__(self, CANAddress1, CANAddress2, inverted=False):
        motor1 = ctre.TalonFX(CANAddress1)
        motor1.setInverted(False)

        motor2 = ctre.TalonFX(CANAddress2)
        motor2.setInverted(False)

        motor2.follow(motor1)            

        return motor1 

    def __set_drivetrain__(self, side, value):
        side.set(ctre._ctre.TalonFXControlMode.PercentOutput, value)
        return None

    def __shift_up__(self):

        forward = wpilib.DoubleSolenoid.Value.kForward

        if self.__current_gear__ != forward:
            # Trigger PCM relay for right side
            self.__rightGearbox__.set(forward)

            # Trigger PCM relay for left side
            self.__leftGearbox__.set(forward)

            # Keep track of current gearbox setting
            self.__current_gear__ = forward

        return None

    def __shift_down__(self):

        reverse = wpilib.DoubleSolenoid.Value.kReverse
        
        if self.__current_gear__ != reverse:
            # Trigger PCM relay for right side
            self.__rightGearbox__.set(reverse)

            # Trigger PCM relay for left side
            self.__leftGearbox__.set(reverse)

            # Keep track of current gearbox setting
            self.__current_gear__ = reverse

        return None 


    def __turn__(self, value):

        # Calculate turning constant to compensate for gear ratio
        if self.current_gear == wpilib.DoubleSolenoid.Value.kReverse:
            distance = self.gear_ratio_low*self.turning_constant*value

        elif self.current_gear == wpilib.DoubleSolenoid.Value.kForward:
            distance = self.gear_ratio_high*self.turning_constant*value

        else:
            distance = 0

        # Move Distance
        self.__move_distance__(self.__leftDrive__, distance)
        self.__move_distance__(self.__rightDrive__, -distance)
        return None

    def __move__(self, value):

        # Calculate turning constant to compensate for gear ratio
        if self.current_gear == wpilib.DoubleSolenoid.Value.kReverse:
            distance = self.gear_ratio_low*self.drive_constant*value

        elif self.current_gear == wpilib.DoubleSolenoid.Value.kForward:
            distance = self.gear_ratio_high*self.drive_constant*value

        else:
            distance = 0

        # Move Distance
        self.__move_distance__(self.__leftDrive__, distance)
        self.__move_distance__(self.__rightDrive__, distance)
        
        return None

    def __move_distance__(self, side, distance):
        side.set(ctre._ctre.TalonFXControlMode.MotionMagic, distance)
        return None

    def __is_finished__(self):
        if self.__rightDrive__.isMotionProfileFinished() and self.__leftDrive__.isMotionProfileFinished():
            return True

        else:
            return False

    def get_queue(self, buffer_size=10):
        self.queue = queue.Queue(buffer_size)
        return self.queue

    def run(self):

        # Start infinite loop
        while True:

            # Take element from queue
            (command, option, value) = self.queue.get()

            if command == 'left drive':
                self.__set_drivetrain__(self.__leftDrive__, value)

            elif command == 'right drive':
                self.__set_drivetrain__(self.__rightDrive__, value)

            elif command == 'shift up':
                self.__shift_up__()

            elif command == 'shift down':
                self.__shift_down__()

            elif command == 'turn':
                self.__turn__(value)

        return None

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