from constants import Constants
from subsystems.Swerve import Swerve
from subsystems.intake import Intake
from subsystems.indexer import Indexer
from subsystems.Shooter import Shooter
from subsystems.Arm import Arm
from subsystems.Vision import Vision
from wpilib import DriverStation
from commands.DynamicShot import DynamicShot
from wpimath.geometry import Pose2d, Rotation2d;
from commands2 import ParallelDeadlineGroup, InstantCommand, InstantCommand, ConditionalCommand, WaitCommand, PrintCommand, SequentialCommandGroup, ParallelCommandGroup, WaitUntilCommand, DeferredCommand
from pathplannerlib.auto import AutoBuilder, PathConstraints
from commands.TurnToTag import TurnToTag
from robotState import RobotState
import math as Math
import json

class autoRunner:
    def __init__(self, s_Swerve: Swerve, s_Intake: Intake, s_Indexer: Indexer, s_Shooter: Shooter, s_Arm: Arm, s_Vision: Vision, targets: json):
        self.Constraints = PathConstraints(
            Constants.AutoConstants.kMaxModuleSpeed,
            Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared,
            Constants.AutoConstants.kMaxAngularSpeedRadiansPerSecond,
            Constants.AutoConstants.kMaxAngularSpeedRadiansPerSecondSquared
        )
        self.s_Swerve = s_Swerve
        self.s_Intake = s_Intake
        self.s_Indexer = s_Indexer
        self.s_Shooter = s_Shooter  
        self.s_Arm = s_Arm
        self.s_Vision = s_Vision
        self.targets = targets
        self.targetsAlliance = (("kBlue") if (targets[0]["x"] < 8.2) else ("kRed"))
        self.targetsFlipped = json.loads(json.dumps(targets))
        for target in self.targetsFlipped:
            target["x"] = 16.541052-target["x"]
            target["theta"] = ((target["theta"] - 180) if (self.targetsAlliance == "kRed") else (180-target["theta"]))

        self.continueTarget = 0
        self.m_robotState : RobotState = RobotState()
        self.m_robotState.initialize(
            lambda: self.s_Swerve.getHeading().degrees(),
            self.s_Arm.getDegrees,
            self.s_Shooter.getVelocity
        )
        self.dynamicShot = DynamicShot(self.s_Swerve, self.s_Vision, self.s_Arm)
        self.currentArmAngle = self.s_Arm.getDegrees()

    def dist(self, x1, y1, x2, y2):
        return Math.sqrt(Math.pow((x2-x1),2) + Math.pow((y2-y1),2))

    def intake(self):
        SequentialCommandGroup(
            self.s_Indexer.indexerIntakeOnce(),
            WaitUntilCommand(self.s_Indexer.getBeamBreakState).withTimeout(4.0).finallyDo(lambda interrupted: self.s_Indexer.stopMotor()),
            self.s_Indexer.instantStop(),
            self.s_Shooter.shootVelocityWithSupplier(
                lambda: 35.0
            )
        ).withTimeout(5.0)
    
    def autoDynamicShot(self):
        return DeferredCommand(
            lambda:
                ParallelDeadlineGroup(
                    WaitUntilCommand(lambda: self.m_robotState.isReadyDynamic(lambda: self.dynamicShot.getInterpolatedArmAngle()) and abs(self.s_Swerve.getHeading().degrees() - self.dynamicShot.getRobotAngle()) <= 0.5)
                    .withTimeout(1.0)
                    .andThen(self.s_Indexer.indexerShoot()),
                    self.s_Arm.servoArmToTargetDynamic(
                        lambda: self.dynamicShot.getInterpolatedArmAngle()
                    ),
                    InstantCommand(lambda: self.m_robotState.m_gameState.setNextShot(Constants.NextShot.DYNAMIC)),
                    self.s_Shooter.shootVelocityWithSupplier(
                        lambda: 35.0
                    ),
                    TurnToTag(
                        self.s_Swerve,
                        lambda: self.dynamicShot.getRobotAngle(),
                        lambda: 0.0,
                        lambda: 0.0,
                        lambda: 0.0,
                        lambda: False
                    )
                ).andThen(self.s_Arm.seekArmZero().withTimeout(0.5))
            )
    
    def autoShootWhenReady(self):
        return DeferredCommand(
            lambda: ConditionalCommand(
                SequentialCommandGroup(
                    self.s_Indexer.indexerShoot(),
                    self.s_Indexer.instantStop(),
                ),
                InstantCommand(),
                lambda: self.s_Indexer.getBeamBreakState()
            )
        )
    
    def runTrajectory(self, num:int, x2:float, y2:float, theta2:float, shot:bool, translate:bool, quickTurn:bool, conditionals:list):
        self.trueValue = [self.s_Intake.getIntakeBeamBreakState(), self.s_Indexer.getBeamBreakState()]
        def doNothing():
            pass
        def dist(x1, y1, x2, y2):
            return Math.sqrt(Math.pow((x2-x1),2) + Math.pow(y2-y1,2))
        trajectory = \
            AutoBuilder.pathfindToPose(Pose2d(x2, y2, Rotation2d((theta2)*Math.pi/180)), self.Constraints, goal_end_vel= 0 if not translate else 4.5, rotation_delay_distance=dist(self.s_Swerve.getPose().X(), self.s_Swerve.getPose().Y(), x2, y2)/3 if not quickTurn else 0)
        if num == self.continueTarget:
            if self.trueValue == [False, False]:
                if conditionals[0] == "continue":
                    self.continueTarget += 1
                    return SequentialCommandGroup(self.intake() if shot else doNothing(), trajectory, SequentialCommandGroup(self.autoDynamicShot(), self.autoShootWhenReady()) if shot else doNothing())
                elif conditionals[0] == "shoot":
                    self.continueTarget += 1
                    return SequentialCommandGroup(SequentialCommandGroup(self.autoDynamicShot(), self.autoShootWhenReady()), self.intake() if shot else doNothing(), trajectory, SequentialCommandGroup(self.autoDynamicShot(), self.autoShootWhenReady()) if shot else doNothing())
                else:
                    numToChange = self.targets[int(conditionals[0].replace("divert", ""))]
                    self.continueTarget = int(numToChange["id"]) + 1
                    return SequentialCommandGroup(AutoBuilder.pathfindToPose(Pose2d(float(numToChange["x"]), float(numToChange["y"]), Rotation2d((float(numToChange["theta"]))*Math.pi/180)), self.Constraints, goal_end_vel= 0 if not (numToChange["type"] == "Translation") else 4.5, rotation_delay_distance=dist(self.s_Swerve.getPose().X(), self.s_Swerve.getPose().Y(), x2, y2)/3 if not self.dist(self.s_Swerve.getPose().X(), self.s_Swerve.getPose().Y(), numToChange["x"], numToChange["y"]) < 3 else 0), SequentialCommandGroup(self.autoDynamicShot(), self.autoShootWhenReady()) if numToChange["type"] == "Shot" else doNothing())
            elif self.trueValue == [False, True]:
                if conditionals[1] == "continue":
                    self.continueTarget += 1
                    return SequentialCommandGroup(self.intake() if shot else doNothing(), trajectory, SequentialCommandGroup(self.autoDynamicShot(), self.autoShootWhenReady()) if shot else doNothing())
                elif conditionals[1] == "shoot":
                    self.continueTarget += 1
                    return SequentialCommandGroup(SequentialCommandGroup(self.autoDynamicShot(), self.autoShootWhenReady()), self.intake() if shot else doNothing(), trajectory, SequentialCommandGroup(self.autoDynamicShot(), self.autoShootWhenReady()) if shot else doNothing())
                else:
                    numToChange = self.targets[int(conditionals[1].replace("divert", ""))]
                    self.continueTarget = int(numToChange["id"]) + 1
                    return SequentialCommandGroup(AutoBuilder.pathfindToPose(Pose2d(float(numToChange["x"]), float(numToChange["y"]), Rotation2d((float(numToChange["theta"]))*Math.pi/180)), self.Constraints, goal_end_vel= 0 if not (numToChange["type"] == "Translation") else 4.5, rotation_delay_distance=dist(self.s_Swerve.getPose().X(), self.s_Swerve.getPose().Y(), x2, y2)/3 if not self.dist(self.s_Swerve.getPose().X(), self.s_Swerve.getPose().Y(), numToChange["x"], numToChange["y"]) < 3 else 0), SequentialCommandGroup(self.autoDynamicShot(), self.autoShootWhenReady()) if numToChange["type"] == "Shot" else doNothing())
            elif self.trueValue == [True, False]:
                if conditionals[2] == "continue":
                    self.continueTarget += 1
                    return SequentialCommandGroup(self.s_ShootersetShooterVelocity(self.s_Shooter.getTargetVelocity()), self.intake() if shot else doNothing(), trajectory, SequentialCommandGroup(self.autoDynamicShot(), self.autoShootWhenReady()) if shot else doNothing())
                elif conditionals[2] == "shoot":
                    self.continueTarget += 1
                    return SequentialCommandGroup(SequentialCommandGroup(self.autoDynamicShot(), self.autoShootWhenReady()), self.intake() if shot else doNothing(), trajectory, SequentialCommandGroup(self.autoDynamicShot(), self.autoShootWhenReady()) if shot else doNothing())
                else:
                    numToChange = self.targets[int(conditionals[2].replace("divert", ""))]
                    self.continueTarget = int(numToChange["id"]) + 1
                    return SequentialCommandGroup(AutoBuilder.pathfindToPose(Pose2d(float(numToChange["x"]), float(numToChange["y"]), Rotation2d((float(numToChange["theta"]))*Math.pi/180)), self.Constraints, goal_end_vel= 0 if not (numToChange["type"] == "Translation") else 4.5, rotation_delay_distance=dist(self.s_Swerve.getPose().X(), self.s_Swerve.getPose().Y(), x2, y2)/3 if not self.dist(self.s_Swerve.getPose().X(), self.s_Swerve.getPose().Y(), numToChange["x"], numToChange["y"]) < 3 else 0), SequentialCommandGroup(self.autoDynamicShot(), self.autoShootWhenReady()) if numToChange["type"] == "Shot" else doNothing())
            else:
                if conditionals[3] == "continue":
                    self.continueTarget += 1
                    return SequentialCommandGroup(self.s_ShootersetShooterVelocity(self.s_Shooter.getTargetVelocity()), self.intake() if shot else doNothing(), trajectory, SequentialCommandGroup(self.autoDynamicShot(), self.autoShootWhenReady()) if shot else doNothing())
                elif conditionals[3] == "shoot":
                    self.continueTarget += 1
                    return SequentialCommandGroup(SequentialCommandGroup(self.autoDynamicShot(), self.autoShootWhenReady()), self.intake() if shot else doNothing(), trajectory, SequentialCommandGroup(self.autoDynamicShot(), self.autoShootWhenReady()) if shot else doNothing())
                else:
                    numToChange = self.targets[int(conditionals[3].replace("divert", ""))]
                    self.continueTarget = int(numToChange["id"]) + 1
                    return SequentialCommandGroup(AutoBuilder.pathfindToPose(Pose2d(float(numToChange["x"]), float(numToChange["y"]), Rotation2d((float(numToChange["theta"]))*Math.pi/180)), self.Constraints, goal_end_vel= 0 if not (numToChange["type"] == "Translation") else 4.5, rotation_delay_distance=dist(self.s_Swerve.getPose().X(), self.s_Swerve.getPose().Y(), x2, y2)/3 if not self.dist(self.s_Swerve.getPose().X(), self.s_Swerve.getPose().Y(), numToChange["x"], numToChange["y"]) < 3 else 0), SequentialCommandGroup(self.autoDynamicShot(), self.autoShootWhenReady()) if numToChange["type"] == "Shot" else doNothing())
            
        else:
            return doNothing()

    def getCommand(self):
        def getAlliance(alliance):
            return alliance == self.targetsAlliance
        def doNothing():
            pass
        return (SequentialCommandGroup(
            self.s_Swerve.setPose(Pose2d(1.36,5.584,Rotation2d(0))), #test blue subwoofer SIM
            #self.s_Swerve.setPose(Pose2d(15.071,5.584,Rotation2d(Math.pi))), #test red subwoofer SIM
            SequentialCommandGroup(self.autoDynamicShot(), self.autoShootWhenReady()) if self.targets[0]["type"] == "Shot" else doNothing(), 
            self.s_Swerve.periodic(),
            ((SequentialCommandGroup(WaitCommand(self.targets[i]["delay"]), self.runTrajectory(self.targets[i]["id"], self.targets[i+1]["x"], self.targets[i+1]["y"], self.targets[i+1]["theta"], self.targets[i+1]["type"] == "Shot", self.targets[i+1]["type"] == "Translation", self.dist(self.targets[i]["x"], self.targets[i]["y"], self.targets[i+1]["x"], self.targets[i+1]["y"]) < 3, self.targets[i]["continueAsPlanned"])),) for i in range(len(self.targets)-1)) if getAlliance("kBlue" if self.s_Swerve.getPose().translation().x < 8.2 else "kRed") else ((SequentialCommandGroup(WaitCommand(self.targetsFlipped[i]["delay"]), self.runTrajectory(self.targetsFlipped[i]["id"], self.targetsFlipped[i+1]["x"], self.targetsFlipped[i+1]["y"], self.targetsFlipped[i+1]["theta"], self.targetsFlipped[i+1]["type"] == "Shot", self.targetsFlipped[i+1]["type"] == "Translation", self.dist(self.targetsFlipped[i]["x"], self.targetsFlipped[i]["y"], self.targetsFlipped[i+1]["x"], self.targetsFlipped[i+1]["y"]) < 3, self.targetsFlipped[i+1]["continueAsPlanned"])),) for i in range(len(self.targetsFlipped)-1)),
        ))