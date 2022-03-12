import rev
import ctre

class __doubleStage__(object):
    def __init__(self, CANAddress1, CANAddress2):
        self.CANAddress1 = CANAddress1
        self.CANAddress2 = CANAddress2
        self.initialize_motors(CANAddress1, CANAddress2)

    
    def initialize_motors(self, CANAddress1, CANAddress2):
        self.motor1 = rev.CANSparkMax(CANAddress1, rev._rev.CANSparkMaxLowLevel.MotorType.kBrushless)
        self.motor1.setInverted(False)

        self.motor2 = rev.CANSparkMax(CANAddress2, rev._rev.CANSparkMaxLowLevel.MotorType.kBrushless)
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
        self.motor1 = rev.CANSparkMax(CANAddress1, rev._rev.CANSparkMaxLowLevel.MotorType.kBrushless)
        self.motor1.setInverted(False)
        return None

    def invert_motor(self):
        self.motor1.setInverted(True)
        return None

    def set_motors(self, value):
        self.motor1.set(value)
        return None






class SHOOTER(object):
    def __init__(self):
        self.__initialize_intake__()
        self.__initialize_shooter__()
        self.__initialize_stage1__()
        self.__initialize_stage2__()
    
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
        self.intake.set(value)
        return None
    
    def __set_shooter__(self, value):
        self.shooter.set(value)
        return None

    def __set_stage1__(self, value):
        self.stage1.set(value)
        return None

    def __set_stage2__(self, value):
        self.stage2.set(value)
        return None
    


    
    def purge(self):
        self.__set_intake__(-1)
        self.__set_shooter__(-1)
        self.__set_stage1__(-1)
        self.__set_stage2__(-1)
        return None

    def advance_ball(self):
        self.__set_stage1__(1)
        self.__set_stage2__(1)
        return None

    def ready_shooter(self):
        self.__set_shooter__(1)
        return None
    
    def disable_intake(self):
        self.__set_intake__(0)
        return None
    
    def enable_intake(self, inverted=False):
        if inverted:
            self.__set_intake__(-1)
        else:
            self.__set_intake__(1)
        return None
    
    def disable_stages(self):
        self.__set_stage1__(0)
        self.__set_stage2__(0)
        return None
    

class ELEVATOR(object):
    def __init__(self):
        return None


class DRIVETRAIN(object):
    def __init__(self):
        self.leftDrive = self.initialize_drive_side(10, 11)
        self.rightDrive = self.initialize_drive_side(12, 13, inverted=True)
        return None
        

    
    def initialize_drive_side(self, CANAddress1, CANAddress2, inverted=False):
        motor1 = ctre.TalonFX(CANAddress1)
        motor1.setInverted(False)

        motor2 = ctre.TalonFX(CANAddress2)
        motor2.setInverted(False)

        motor2.follow(self.motor1)            

        return motor1

    def __set_drive_side__(self, side, value):
        side.set(ctre._ctre.TalonFXControlMode.PercentOutput, value)
    


