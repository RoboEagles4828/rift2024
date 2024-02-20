from commands2.subsystem import Subsystem
# from commands2.subsystem import Command
from commands2 import WaitCommand
from commands2 import WaitUntilCommand
import wpimath.filter
import wpimath
import wpilib
import phoenix5
import math

class Arm(Subsystem):

    # 
    #    The motion magic parameters are configured in slot 0. Slot 1 is configured
    #    for velocity control for zeroing.
    #    


    def __init__(self):
        self.kArmMotorCANId = 5
        self.kMeasuredTicksWhenHorizontal = 0
        self.kEncoderTickPerEncoderRotation = 4096
        self.kEncoderToArmGearRatio = 1.0
        self.kEncoderTicksPerArmRotation = self.kEncoderTickPerEncoderRotation * self.kEncoderToArmGearRatio
        self.kEncoderTicksPerDegreeOfArmMotion = self.kEncoderTicksPerArmRotation / 360.0
        self.kMotionMagicSlot = 0
        self.kVelocitySlot = 1
        self.MaxGravityFF = 0.26 # In percent output [1.0:1.0]
        self.kF = 0.2
        self.kPMotionMagic = 4.0
        self.kPVelocity = 0.8
        self.kIMotionMagic = 0.0
        self.kDMotionMagic = 0.0
        self.kCruiseVelocity = 1000.0 # ticks per 100ms
        self.kMaxAccel = 1000.0 # Accel to cruise in 1 sec
        self.kServoToleranceDegrees = 1.0 # +/- 1.0 for 2.0 degree window
        # Velocity for safely zeroing arm encoder in native units (ticks) per 100ms
        self.kZeroEncoderVelocity = -self.kEncoderTicksPerDegreeOfArmMotion * 5.0
        self.kZeroingWaitForMoveSec = 2.0
        self.ZeroingVelocityTolerance = 10.0
        

        self.armMotor = phoenix5.TalonSRX(self.kArmMotorCANId)

        # True when servo control active and false otherwise.
        self.isServoControl = False
        # The last requested servo target for target checking.
        self.lastServoTarget = 0.0
        self.kRestingAtZero = False

        self.armMotor.setSensorPhase(True)
        self.armMotor.config_kP(self.kMotionMagicSlot, self.kPMotionMagic)
        self.armMotor.config_kI(self.kMotionMagicSlot, self.kIMotionMagic)
        self.armMotor.config_kD(self.kMotionMagicSlot, self.kDMotionMagic)
        self.armMotor.config_kF(self.kMotionMagicSlot, self.kF)
        self.armMotor.configMotionCruiseVelocity(self.kCruiseVelocity)
        self.armMotor.configMotionAcceleration(self.kMaxAccel)

        self.armMotor.config_kP(self.kVelocitySlot, self.kPVelocity)
        self.armMotor.config_kI(self.kVelocitySlot, 0.0)
        self.armMotor.config_kD(self.kVelocitySlot, 0.0)
        self.armMotor.config_kF(self.kVelocitySlot, self.kF)
        

    #     
    #     Creates a command to seek the arm's zero position. This command is designed
    #     to always be used for returning to and settling at zero. It should be the
    #     subsystem's default command. The encoder will be reset to 0.
    #     
    #     @return a command that will move the arm toward 0, and stop when stalled.
    #    
    def seekArmZero(self):
        
        return self.runOnce(lambda: self.selectPIDSlot(self.kVelocitySlot)).andThen(self.armMotor.set(
            phoenix5.ControlMode.Velocity,
            self.kZeroEncoderVelocity,
            phoenix5.DemandType.ArbitraryFeedForward,
            self.calculateGravityFeedForward())) \
            .raceWith(WaitCommand(self.kZeroingWaitForMoveSec) \
            .andThen(self.detectStallAtHardStop())) \
            .andThen(self.restingAtZero()) \
            .withName("seekArmZero")

    #     
    #     Creates a command to detect stall at the hard stop during
    #     {@link #seekArmZero()}.
    #     
    #     @return the hard stop detection command.
    #    
    def detectStallAtHardStop(self):
        stallDebouncer = wpimath.filter.Debouncer(1.0, wpimath.filter.Debouncer.DebounceType.kRising)  
        return WaitUntilCommand(lambda: abs(self.armMotor.getSelectedSensorVelocity(self.kVelocitySlot) - 0.0) < self.ZeroingVelocityTolerance)
        

    #     
    #     Should only be called as the last step of {@link #seekArmZero()}. The encoder
    #     is reset to 0.
    #     
    #     @return a command to rest at the hard stop at 0 and on target.
    #    
    def restingAtZero(self):
        return self.runOnce(lambda: self.setRestingAtZero(True)) \
        .andThen(self.run(lambda: self.armMotor.set(phoenix5.ControlMode.Velocity,0.0)).withTimeout(1.0)) \
        .andThen(lambda: self.hardSetEncoderToZero() ) \
        .andThen(self.run(lambda: self.armMotor.set(phoenix5.ControlMode.Velocity,-0.1))) \
        .finallyDo(lambda interrupted: self.setRestingAtZero(False))
    
    def setRestingAtZero(self, restAtZero):
        self.kRestingAtZero = restAtZero

    def hardSetEncoderToZero(self):
        self.armMotor.setSelectedSensorPosition(0)

    
    #     
    #     Creates a command to servo the arm to a desired angle. Note that 0 is
    #     parallel to the ground. The entire operation is run with
    #     {@link m_isServoControl} set to true to enable on target checking. See
    #     {@link #isServoOnTarget(double)}.
    #     
    #     <p>
    #     If the target is 0 or less, the command from {@link #seekArmZero()} is
    #     returned.
    #     
    #     @param degrees the target degrees from 0. Must be positive.
    #     @return a command that will servo the arm and will not end until interrupted
    #             by another command.
    #    
    def servoArmToTarget(self, degrees) :
        if (degrees <= 0.0):
            return self.seekArmZero()

        targetSensorUnits = degrees * self.kEncoderTicksPerDegreeOfArmMotion
        return self.runOnce(lambda: self.initializeServoArmToTarget(degrees)) \
        .andThen(lambda: self.run(self.armMotor.set(
            phoenix5.ControlMode.MotionMagic,
            targetSensorUnits,
            phoenix5.DemandType.ArbitraryFeedForward,
            self.calculateGravityFeedForward()))) \
        .finallyDo(lambda: self.setServoControl(False)) \
        .withName("servoArmToTarget: " + str(degrees))
  
    def initializeServoArmToTarget(self, degrees):
        self.lastServoTarget = degrees
        self.setServoControl(True)
        self.selectPIDSlot(self.kMotionMagicSlot)

    def setServoControl(self, servoControl):
        self.isServoControl = servoControl


    #      
    #     This method should rarely be used. It is for pure manual control (no encoder
    #     usage) which should be avoided.
    #    
    #     @param percentOutput the commanded output [-1.0..1.0]. Positive is up.
    #     @return a command that drives the arm via double supplier.
    #    
    def moveArm(self, percentOutput):
        return self.run(lambda: self.armMotor.set(percentOutput() * 0.4)) \
        .withName("moveArm")
  


    #    
    #    Assuming a properly zeroed arm (0 degrees is parallel to the ground), return
    #    the current angle adjusted gravity feed forward.
    #     
    #    @return the current angle adjusted gravity feed forward.
    #    
    def calculateGravityFeedForward(self):
        degrees = self.getDegrees()
        radians = degrees * (math.pi/180.0)
        cosineScalar = math.cos(radians)
        return self.MaxGravityFF * cosineScalar
  
    #     
    #     Assuming a properly zeroed arm (0 degrees is parallel to the ground), return
    #     the current arm angle.
    #     
    #     @return the current arm angle in degrees from 0.
    #    
    def getDegrees(self):
        currentPos = self.armMotor.getSelectedSensorPosition()
        return (currentPos - self.kMeasuredTicksWhenHorizontal) / self.kEncoderTicksPerDegreeOfArmMotion


    #    
    #    Returns true if the arm is under servo control and we are close to the last
    #    requested target degrees.
    #     
    #    @return true if under servo control and close, false otherwise.
    #    
    def isServoOnTarget(self):
        return self.kRestingAtZero \
            or (self.isServoControl \
                and abs(self.lastServoTarget - self.getDegrees()) > self.kServoToleranceDegrees)



    #         
    #    Selects the specified slot for the next PID controlled arm movement. Always
    #    selected for primary closed loop control.
    #     
    #    @param slot the PID slot
    #    
    def selectPIDSlot(self, slot):
        self.armMotor.selectProfileSlot(slot, 0)
        

    #    
    #    Updates the dashboard with critical arm data.
    #    
  
    def periodic(self) :
        # TODO reduce this to essentials.
        wpilib.SmartDashboard.putNumber("Arm degrees", self.getDegrees())
        wpilib.SmartDashboard.putNumber("Arm current", self.armMotor.getStatorCurrent())
        wpilib.SmartDashboard.putBoolean("Arm on target", self.isServoOnTarget())
        currentCommand = self.getCurrentCommand()
        wpilib.SmartDashboard.putString("Arm command", currentCommand.getName() if currentCommand is not None else "<null>")
        wpilib.SmartDashboard.putNumber("Arm zeroing velocity", self.armMotor.getSelectedSensorVelocity(self.kVelocitySlot))
        wpilib.SmartDashboard.putBoolean("Arm resting", self.kRestingAtZero)
        wpilib.SmartDashboard.putBoolean("Servo control", self.isServoControl)