from constants import Constants
from wpilib import DriverStation


class GameState:
    """
    The single game state object holds information on the current state of our
    game play. It includes that next desired shot and if we do or do not
    currently hold a note.
    """

    def __new__(cls):
        if not hasattr(cls, "instance"):
            cls.instance = super(GameState, cls).__new__(cls)
        return cls.instance

    m_hasNote = True
    """ Do we have a note onboard? Start auto holding a note. """

    m_nextShot = Constants.NextShot.SPEAKER_CENTER
    """ What is our next planned shot? Default for basic autos. """

    # Sets the next shot to take. If null is passed, the shot is reset to its
    # initial speaker center state. The next shot setting is never allowed to be
    # None.
    #
    # nextShot must be an instance of Constants.NextShot
    def setNextShot(self, nextShot):
        """
        Sets the next shot to take. If null is passed, the shot is reset to its
        initial speaker center state. The next shot setting is never allowed to be
        None.

        :param nextShot: must be an instance of Constants.NextShot
        """
        if isinstance(nextShot, Constants.NextShot):
            self.m_nextShot = nextShot
        else:
            self.m_nextShot = Constants.NextShot.SPEAKER_CENTER

    def getNextShot(self) -> Constants.NextShot:
        """
        Return the next desired shot. Never None.
        """
        return self.m_nextShot

    def getNextShotRobotAngle(self) -> float:
        """
        Return the angle the robot needs to be turned to for the next shot. This
        value is alliance adjusted. If the FMS is misbehaving, we assume blue.
        """
        alliance = DriverStation.getAlliance()
        nextShot = self.getNextShot()
        if alliance == DriverStation.Alliance.kRed:
            return nextShot.m_redSideBotHeading
        return nextShot.m_blueSideBotHeading
    
    def getNextShotTagID(self) -> int:
        """
        Return the tag ID for the next shot. This value is alliance adjusted. If
        the FMS is misbehaving, we assume blue.
        """
        alliance = DriverStation.getAlliance()
        nextShot = self.getNextShot()
        if alliance == DriverStation.Alliance.kRed:
            return nextShot.m_redTagID
        return nextShot.m_blueTagID

    def setHasNote(self, hasNote):
        """
        Generally called by the indexer with True and the shooter with False.

        :param hasNote: the new setting for has note.
        """
        self.m_hasNote = hasNote

    def hasNote(self) -> bool:
        """
        Return true if the robot is currently holding a note. False, otherwise.
        """
        return self.m_hasNote
