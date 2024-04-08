import math

from gameState import GameState


class RobotState:
    """
    The single robot can evaluate if the robot subsystems are ready to
    execute a game task.
    """

    kRobotHeadingTolerance = 2.0
    kArmAngleTolerance = 1.0
    kShooterVelocityTolerance = 3.0

    m_gameState = GameState()

    def __new__(cls):
        """
        To initialize the singleton RobotState, call this from
        RobotContainer soon after creating subsystems and
        then immediately call initialize() the returned instance.

        Other clients of the singleton only call this constructor.
        """
        if not hasattr(cls, "instance"):
            cls.instance = super(RobotState, cls).__new__(cls)
        return cls.instance

    def initialize(
        self, robotHeadingSupplier, armAngleSupplier, shooterVelocitySupplier
    ):
        """
        Only called from RobotContainer after the constructor to complete
        initialization of the singleton.

        :param robotHeadingSupplier: a supplier of the robots current heading.
        :param armAngleSupplier: a supplier of the current arm angle.
        :param shooterVelocitySupplier: a supplier of the current shooter velocity.
        """
        self.m_robotHeadingSupplier = robotHeadingSupplier
        self.m_armAngleSupplier = armAngleSupplier
        self.m_shooterVelocitySupplier = shooterVelocitySupplier

    def isclose(self, a, b, tolerance) -> bool:
        return abs(a - b) < tolerance

    def isShooterReady(self) -> bool:
        """
        Return true if the current arm angle and shooter velocity are both within
        tolerance for the needs of the next desired shot.
        """
        shot = self.m_gameState.getNextShot()
        return self.isclose(
            shot.m_shooterVelocity,
            self.m_shooterVelocitySupplier(),
            self.kShooterVelocityTolerance,
        )
    
    def isArmReady(self) -> bool:
        shot = self.m_gameState.getNextShot()
        return self.isclose(
            shot.m_armAngle,
            self.m_armAngleSupplier(),
            self.kArmAngleTolerance
        )
    
    def isArmAndShooterReady(self) -> bool:
        return self.isShooterReady() and self.isArmReady()

    def isRobotReady(self) -> bool:
        """
        Return true if isShooterReady and the robot is turned correctly for the next shot.
        """
        return self.isArmAndShooterReady() and self.isclose(
            self.m_gameState.getNextShotRobotAngle(),
            self.m_robotHeadingSupplier(),
            self.kRobotHeadingTolerance,
        )

    def isReadyToIntake(self) -> bool:
        """
        On our robot, the intake is physically dependent on the arm position for
        intaking to work.

        :return: True if we do not have a note and the arm is in the proper
        intaking position.
        """
        return (not self.m_gameState.hasNote()) and self.isclose(
            0.0, self.m_armAngleSupplier(), self.kArmAngleTolerance
        )
    
    def isReadyDynamic(self, degreesSup) -> bool:
        arm = self.isclose(
            degreesSup(),
            self.m_armAngleSupplier(),
            self.kArmAngleTolerance
        )

        shooter = self.isclose(
            35.0,
            self.m_shooterVelocitySupplier(),
            self.kShooterVelocityTolerance
        )

        return arm and shooter