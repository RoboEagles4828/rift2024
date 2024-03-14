class Conversions:

    """
    :param wheelRPS: wheel rotations per second
    :param circumference: wheel circumference
    :returns: wheel meters per second
    """
    @staticmethod
    def RPSToMPS(wheelRPS: float, circumference: float):
        wheelMPS = wheelRPS * circumference
        return wheelMPS

    """
    :param wheelMPS: wheel meters per second
    :param circumference: wheel circumference
    :returns: wheel rotations per second
    """
    @staticmethod
    def MPSToRPS(wheelMPS: float, circumference: float):
        wheelRPS = wheelMPS / circumference
        return wheelRPS

    """
    :param wheelRotations: Wheel Position (in Rotations)
    :param circumference: Wheel Circumference (in Meters)
    :returns: Wheel Distance (in Meters)
    """
    @staticmethod
    def rotationsToMeters(wheelRotations: float, circumference: float):
        wheelMeters = wheelRotations * circumference
        return wheelMeters

    """
    :param wheelMeters: Wheel Distance (in Meters)
    :param circumference: Wheel Circumference (in Meters)
    :returns: Wheel Position (in Rotations)
    """
    @staticmethod
    def metersToRotations(wheelMeters: float, circumference: float):
        wheelRotations = wheelMeters / circumference
        return wheelRotations
