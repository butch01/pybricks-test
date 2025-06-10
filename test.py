from pybricks.hubs import InventorHub
from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch
from pybricks.parameters import Color, Axis


class log:
    """
    Static logging class.
    """

    loglevel = 0
    LOGLEVEL_DEBUG = 0
    LOGLEVEL_INFO = 1
    LOGLEVEL_ERROR = 2

    @staticmethod
    def debug(message: str):
        if log.loglevel <= log.LOGLEVEL_DEBUG:
            print(f"{stopWatch.time():07d} D: {message}")

    @staticmethod
    def info(message: str):
        if log.loglevel <= log.LOGLEVEL_INFO:
            print(f"{stopWatch.time():07d} I: {message}")

    @staticmethod
    def error(message: str):
        if log.loglevel <= log.LOGLEVEL_ERROR:
            print(f"{stopWatch.time():07d} E: {message}")

class PIDController:
    def __init__(self, Kp: float, Ki: float, Kd: float, deadzone: float):
        """ Initialisiert die PID-Parameter und speichert vorherige Werte für die Berechnung. """
        self.Kp = Kp  # Proportionaler Faktor - reagiert direkt auf den Fehler
        self.Ki = Ki  # Integraler Faktor - summiert Fehler über die Zeit für langfristige Korrekturen
        self.Kd = Kd  # Differentieller Faktor - reagiert auf Änderungen des Fehlers für Stabilisierung
        self.last_error = 0  # Speichert den Fehler aus der letzten Berechnung
        self.integral = 0  # Akkumuliert Fehler über die Zeit
        self.deadzone = deadzone # do not correct if error is within the deadzone

    def compute(self, target, current):
        """ Berechnet die Korrektur basierend auf PID-Regelung. """
        error = target - current  # Berechnet den Unterschied zwischen Soll- und Ist-Wert
    
        self.integral += error  # Fehler summieren für den Integralanteil
        derivative = error - self.last_error  # Berechnet die Änderung des Fehlers (Differentialanteil)
        self.last_error = error  # Speichert aktuellen Fehler für nächste Berechnung
        if abs(error) > self.deadzone: # großer Fehler, nicht in Deadzone
            output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative  # PID-Gesamtformel
        else:
            output = 0 # in deadzone, kein Fehler
        return output  # Gibt den berechneten Korrekturwert zurück

    def reset(self):
        """ Setzt die gespeicherten Werte des PID-Reglers zurück. """
        self.last_error = 0  # Fehler zurücksetzen
        self.integral = 0  # Integralwert zurücksetzen

class myDriveBase(DriveBase):
    """
    Extension of class DriveBase.
    adds special functions like driveStraightToTarget(), calculateHeadingCorrection(), checkDistance(), ...
    """
    
    def __init__(self, motorLeft: Motor, motorRight: Motor, wheelDiameter: float, axleTrack: float, speedStraightDefault: float, accStraightDefault: float, speedTurnDefault: float, accTurnDefault: float):
        """
        Constructor
        """

        super().__init__(motorLeft, motorRight, wheelDiameter, axleTrack)  # calls superclass constructor
        # set default values for class variables
        self.myMotorLeft = motorLeft
        self.wheelDiameter = wheelDiameter
        self.myMotorRight = motorRight
        self.speedStraightDefault = speedStraightDefault
        self.accStraightDefault = accStraightDefault
        # # currently not using pid controller
        # self.pidHeading = PIDController(1.0, 0.1, 0.01, 1)  # Beispielwerte für die PID-Parameter
        # self.pidHeading = PIDController(0.5, 0.1, 2, 0)  # Beispielwerte für die PID-Parameter
        
        # config values for correction
        self.correctionLimitMin = 5
        self.correctionLimitMax = 200
        self.correctionDeadzone = 0.5
        self.correctionFactor = 5

        # setting default for breaking into slow speed area
        self.slowSpeedDeacceleration = int(3000) # set breaking deacceleration to come to the slow speed area

        # defining event variables
        self.stopEventReached = False
        self.eventBreakingPointReached = False  # informs if the breaking point is reached
        self.isEventSlowSpeedBreakingReached = False # informs, if the pint for start breaking for slow speed area is reached
        self.isEventDistanceReached = False # informs if the distance is reached
        self.isEventStoppingReached = False # informs if the robot is going to stop
        
        # store last 5 measured speeds for averaging
        self.lastSpeedList = []
        self.speedListMaxLength = 5  # maximum length of the speed list
        self.currentAvgSpeed = 0 # current average speed, calculated from the last speedListMaxLength speeds

    def rememberSpeed(self, speed: int):
        """Fügt eine neue Geschwindigkeit zur Liste hinzu und hält die Länge bei maximal speedListMaxLength."""
        self.lastSpeedList.append(speed)
        if len(self.lastSpeedList) > self.speedListMaxLength:
            self.lastSpeedList.pop(0)

    def calculateAverageRememberedSpeed(self):
        """Berechnet den Mittelwert der letzten 5 Geschwindigkeiten und speichert ihn als self.currentAvgSpeed."""
        if not self.lastSpeedList:
            return 0
        self.currentAvgSpeed = sum(self.lastSpeedList) / len(self.lastSpeedList)

    def calculateHeadingCorrection(self, targetHeading: float) -> tuple[float, float]:
        """
        Berechnet die Geschwindigkeitskorrektur basierend auf dem Heading-Delta.

        :param targetHeading: Die Zielausrichtung (in Grad), auf die sich das Fahrzeug ausrichten soll.
        :return: Ein Tupel bestehend aus:
                - correction: Die berechnete Geschwindigkeitskorrektur für die Motoren.
                - headingDelta: Der Unterschied zwischen aktuellem und Ziel-Heading.
        """

        # calculate error / delta in heading
        headingDelta = (targetHeading - hub.imu.heading())
        correction = 0
        if abs(headingDelta) > self.correctionDeadzone:
                correction = headingDelta * self.correctionFactor # multiply error with factor to get higher changes
                # limit the correction factor
                if correction > self.correctionLimitMax:
                    correction = self.correctionLimitMax
                elif correction < -1 * self.correctionLimitMax:
                    correction = -1 * self.correctionLimitMax
                elif correction > (-1 * self.correctionLimitMin) and correction < 0:
                    correction = -1 * self.correctionLimitMin
                elif correction < self.correctionLimitMin and correction > 0:
                    correction = self.correctionLimitMin
        return correction, headingDelta  # Rückgabe als Tupel

    def isStopped(self):
        """
        checks if both wheels are stopped
        """

        output = False
        if (self.myMotorLeft.speed() == 0 and self.myMotorRight.speed() == 0):
            output = True
        return output

    def degrees_to_cm(self, degrees: float) -> float:
        """
        Wandelt eine Drehbewegung in cm um.
        :param degrees: Anzahl der Grad.
        :return: Entsprechende Strecke in cm.
        """

        wheel_diameter = self.wheelDiameter  # Muss als Klassenvariable existieren!
        circumference = 3.1415 * wheel_diameter  # Umfang des Rads
        return (degrees * circumference) / 360

    def checkBreakingPoint(self, distance: float, acc: int) -> bool:
        """
        Überprüft, ob das Fahrzeug die angegebene Distanz erreicht oder unterschritten hat. Dabei wird der Bremsweg mit der gegebenen Bremsbeschleunigung acc berücksichtigt.
        
        Voraussetzung:  self.currentAvgSpeed wurde bereits berechnet.

        :param distance: Die Zielentfernung (in Einheiten, z. B. cm), bei der das Event ausgelöst werden soll.#
        :param acc: Bremsbeschleunigung, die bei den Motoren gesetzt ist.
        :return: `True`, wenn die aktuelle Distanz kleiner oder gleich `distance` ist, andernfalls `False`.
        """

        # calculate break distance (when using current acc)

        # Aktuelle Geschwindigkeit in cm/s umrechnen
        currentSpeed_cm = self.degrees_to_cm(self.currentAvgSpeed)

        # Bremsbeschleunigung ebenfalls umrechnen
        acc_cm = self.degrees_to_cm(abs(acc))

        # Berechnung der Bremsdistanz in cm
        breakingDistance = (currentSpeed_cm ** 2) / (2 * acc_cm)
        
        # Überprüfung, ob Fahrzeug anhalten soll
        output = False
        if self.distance() + breakingDistance >= distance:
            output = True
            log.debug(f"checkDistance: targDist = {distance}, driven= {self.distance()} breaking= {breakingDistance} speed= {self.currentAvgSpeed} reached={output}")
        return output

    def driveStraightToTarget(self, distance: float, targetHeading: float, speed: int = None, acc: int = None, stop: Stop = Stop.HOLD, slowDistance: float = None, slowSpeed: int = None, slowDistanceBreakAcc: int = None ):
        """
        Bewegt das Fahrzeug eine bestimmte Distanz geradeaus, während es eine Zielausrichtung (Heading) beibehält.

        :param distance: Die gewünschte Fahrstrecke in Einheiten (z. B. cm).
        :param targetHeading: Der gewünschte Ziel-Heading-Winkel (in Grad), den das Fahrzeug beibehalten soll.
        :param speed: Die gewünschte Geschwindigkeit der Motoren (optional). Falls None, wird der Standardwert `self.speedStraightDefault` verwendet.
        :param acc: Die gewünschte Beschleunigung der Motoren (optional). Falls None, wird der Standardwert `self.accStraightDefault` verwendet.
        :param stop: Die Stopp-Methode nach Erreichen des Ziels (z. B. `Stop.HOLD` oder `Stop.COAST`). Setze auf None, damit der Motor weiterdreht, aber die Funktion beendet wird.
        :param slowDistance: Ab dieser Entfernung soll eine niedrigere Geschwindigkeit verwendet werden (optional)
        :param slowSpeed: Geschwindigkeit für den langsamfahrenden Teil
        :param slowDistanceBreakAcc:
        """
        log.debug(f"driveStraightToTarget({distance}, {targetHeading}, {speed}, {acc}, {stop})" )
        # set default for speed, acc,slowDistanceBreakAcc if not configured. Take Values from instance variables
        if speed is None:
            speed = self.speedStraightDefault
        if acc is None:
            acc = self.accStraightDefault
        if slowDistanceBreakAcc is None:
            slowDistanceBreakAcc = self.slowSpeedDeacceleration
        
        # configure acceleration for both motors
        self.myMotorLeft.control.limits(acceleration=acc)
        self.myMotorRight.control.limits(acceleration=acc)
        # start the motor, is respecting the acceleration limits which have been configured for motor
        log.debug(f"driveStraightToTarget: start both motors with speed {speed:03d}")
        self.myMotorLeft.run(speed)
        self.myMotorRight.run(speed)

        isSlowSpeedBreaking = False
        self.stopEventReached = False
        
        # run as long as neither self.stopEventReached is True nor self.isEventDistanceReached is True
        while not self.stopEventReached and not self.isEventDistanceReached:
            # correct the direction if needed
            correction, headingDelta = self.calculateHeadingCorrection(targetHeading)

            # remember speed
            self.rememberSpeed((self.myMotorLeft.speed() + self.myMotorRight.speed()) / 2)
            self.calculateAverageRememberedSpeed()

            
            if not self.isEventSlowSpeedBreakingReached and not self.isEventStoppingReached and not self.eventBreakingPointReached:
                # we are in normal speed area, so we can correct use speed as basis
                self.myMotorLeft.run(speed + correction)
                self.myMotorRight.run(speed - correction)
            elif self.isEventSlowSpeedBreakingReached:
                # we are in slow speed area, so we can correct use slowSpeed as basis
                self.myMotorLeft.run(slowSpeed + correction)
                self.myMotorRight.run(slowSpeed - correction)
            # in other cases, do not correct heading anymore

            # process breaking into slow speed area (only check if slowDistance is set)
            if slowDistance is not None and slowSpeed is not None and isSlowSpeedBreaking == False:
                if self.checkBreakingPoint(slowDistance, slowDistanceBreakAcc):
                    self.myMotorLeft.control.limits(acceleration=slowDistanceBreakAcc)
                    self.myMotorRight.control.limits(acceleration=slowDistanceBreakAcc)
                    self.myMotorLeft.run(slowSpeed)
                    self.myMotorRight.run(slowSpeed)
                    isSlowSpeedBreaking = True
                    self.EventSlowSpeedBreakingReached = True
                    log.debug(f"driveStraightToTarget: slow speed breaking at {self.distance()} with speed {slowSpeed} breakAcc {slowDistanceBreakAcc}")
                    
            # only check distance if we have a distance limit
            if distance != 0:
                if self.checkBreakingPoint(distance, acc) and not self.eventBreakingPointReached:
                    # if stop is None, just end this function and keep current speed
                    if stop is not None:
                        self.myMotorLeft.run(0)
                        self.myMotorRight.run(0)
                        log.debug(f"driveStraightToTarget: breaking point reached at {self.distance()} with speed {self.currentAvgSpeed} acc {acc}, stop(0) called")
                        # self.stopEventReached = True    # remember that we have a stop event here
                    self.eventBreakingPointReached = True

            # immediately stop if distance is reached
            currentDistanceDriven = self.distance()
            beforeStopMultiplier = 0.05
            if currentDistanceDriven + self.currentAvgSpeed * beforeStopMultiplier >= distance and distance != 0:
                log.info(f"driveStraightToTarget: distance reached {currentDistanceDriven} + {self.currentAvgSpeed * beforeStopMultiplier} >= {distance}")
                self.isEventDistanceReached = True
                if stop is not None:
                    self.myMotorLeft.stop()
                    self.myMotorRight.stop()
                    self.stopEventReached = True

            log.info(
                f"driveStraightToTarget: headingDelta: {headingDelta} " +
                f"correction {correction} speedL {self.myMotorLeft.speed()} " +
                f"speedR {self.myMotorRight.speed()} " +
                f"speedAvg {self.currentAvgSpeed}  dist {self.distance()} "
            )
                
        
        # wait for full stop, as long stop is not set to None
        # if stop is set to None, just end the function
        if stop is not None:
            while not self.isStopped():
                wait(10) # wait until motors are really stopped            

class MotorAsServo:
    """Eine Klasse für einen Motor als Servo mit Positionsgrenzen."""

    def __init__(self, motorPort: Port, motorDirection: Direction = Direction.CLOCKWISE, positionLimitLow: int = -179, positionLimitHigh: int = 180, debug: bool = False, name: str = ""):
        """
        Konstruktor zur Initialisierung der Positionsgrenzen des Motors.
        
        :param name: Name des Motors, default angegebener Port
        :param motorPort: Port des Motors (z.B. Port.A).
        :param motorDirection: Direction des Motors (z.B. Direction.CLOCKWISE).
        :param positionLimitLow: Die untere Positionsgrenze des Motors (z.B. 0).
        :param positionLimitHigh: Die obere Positionsgrenze des Motors (z.B. 180).
        :param debug: schaltet Debugausgaben ein oder aus. Default False (z.B. False).
        """
        
        # Überprüfung der Grenzwerte
        if positionLimitLow >= positionLimitHigh:
            raise ValueError("positionLimitLow muss kleiner als positionLimitHigh sein!")
        
        # Wenn kein Name gesetzt ist, setze motorPort als Name
        if name == "" :
            self.name = str(motorPort)
        else:
            self.name = name

        self.motor = Motor(motorPort, motorDirection)
        self.motor.reset_angle(None)
        self.motorLimitLow = positionLimitLow
        self.motorLimitHigh = positionLimitHigh

        

    def moveToPosition(self, angleTarget: int, speed: int = 500, stop: Stop = Stop.COAST, wait: bool = True):
        """
        Bewegt den Motor zur gewünschten Position unter Berücksichtigung von Begrenzungen.

        :param angleTarget: Zielwinkel des Motors.
        :param speed: Geschwindigkeit der Motorbewegung (Standard: 500).
        :param stop: Stop-Modus des Motors (Standard: Stop.COAST).
        :param wait: Gibt an, ob gewartet werden soll, bis die Bewegung abgeschlossen ist (Standard: True).
        """
        log.info(f"{self.name}: moveToPosition: (t={angleTarget:04d}, s={speed:04d}, {stop}, w={wait}")
        self.motor.reset_angle()
        # limit arm position to match builiding constraints
        if angleTarget < self.motorLimitLow:
            angleTarget = self.motorLimitLow
        elif angleTarget > self.motorLimitHigh:
            angleTarget = self.motorLimitHigh

        # fix motor direction when motor ppsition change would run outside the limit
        currentAngle: int = self.motor.angle()
        if currentAngle < self.motorLimitLow:
            currentAngle = currentAngle + 360
        if currentAngle > self.motorLimitHigh:
            currentAngle = currentAngle - 360

        angleDiff = angleTarget - currentAngle
        log.debug(f"{self.name}: moveToPosition: orig curr={currentAngle:04d} targ={angleTarget:04d} diff={angleDiff:04d}")

        # check if rotation would lead through out of bounds
        if angleDiff + currentAngle > self.motorLimitHigh:
            log.debug("1")
            angleDiff = 360 - angleDiff
        elif angleDiff + currentAngle < self.motorLimitLow:
            log.debug("2")
            angleDiff = -360 + angleDiff
        log.debug(f"{self.name}: moveToPosition: corr curr={currentAngle:04d} targ={angleTarget:04d} diff={angleDiff:04d}")
    

        self.motor.run_angle(speed, angleDiff, stop, wait)
        log.debug(f"{self.name}: moveToPosition: END curr={self.motor.angle():04d} targ={angleTarget:04d} ")

class Constants:
    COLOR_SENSOR_DOWN           = int(-5)
    COLOR_SENSOR_PROBE_DRIVE_BY = int(90)
    COLOR_SENSOR_PROBE_UP       = int(-140)
    ARM_PROBE_DOWN              = int(-40)
    ARM_PROBE_UP                = int(40)
    ARM_ROVER_UP                = int(170)
    ARM_ROVER_DOWN              = int(190)
    
    class Color:
        class Black:
            reflectionMin = int (1)
            reflectionMax = int (13)
            hsvMin        = [ int(230), int(13), int (7)]
            hsvHMax       = [ int(300), int(31), int (18) ]

        class White:
            reflectionMin = int (75)
            reflectionMax = int (100)
            hsvMin        = [ int(205), int(19), int (95)]
            hsvHMax       = [ int(220), int(25), int (100) ]




# def turn(angle: int, speed: int = speed_turn, acc: int = acc_turn, stop: Stop = Stop.HOLD, wait: bool = True, gyro: bool = False):
#     """
#     Führt eine Drehung der Roboterbasis um den angegebenen Winkel aus.

#     Parameter:
#     angle (int): Drehwinkel in Grad (positiv für Rechtsdrehung, negativ für Linksdrehung).
#     speed (int): Maximale Drehgeschwindigkeit.
#     acc (int): Beschleunigung für die Drehbewegung.
#     stop (Stop): Stopp-Modus nach der Drehung (z. B. Bremsen oder Ausrollen).
#     wait (bool): Ob die Funktion warten soll, bis die Drehung abgeschlossen ist.
#     gyro (bool): Ob das Gyroskop zur Stabilisierung der Drehung verwendet wird (falls vorhanden).

#     Rückgabe:
#     None
#     """
#     # default (7558, 0, 1889, 3, 6)
#     # driveBase.heading_control.pid (7558, 1000, 1889, 1, 6)
#     driveBase.heading_control.pid(7558, 0, 1889, 1, 6)
#     driveBase.settings(speed_straight, acc_straight, speed_turn, acc_turn)
#     driveBase.turn(angle, stop, wait)

# def driveStraight(distance: int, speed: int = speed_straight, acc: int = acc_straight, stop: Stop = Stop.HOLD, wait: bool = True, gyro: bool = False):
#     """
#     Fährt die Roboterbasis eine gerade Strecke mit den angegebenen Parametern.

#     Parameter:
#     distance (int): Die Strecke, die der Roboter zurücklegen soll (in Millimetern).
#     speed (int): Maximale Fahrgeschwindigkeit.
#     acc (int): Beschleunigung für die Fahrt.
#     stop (Stop): Stopp-Modus nach der Fahrt (z. B. Bremsen oder Ausrollen).
#     wait (bool): Ob die Funktion warten soll, bis die Bewegung abgeschlossen ist.
#     gyro (bool): Ob das Gyroskop zur Stabilisierung der Bewegung verwendet wird (standardmäßig aktiviert).

#     Rückgabe:
#     None
#     """
#     # default (7558, 0, 1889, 3, 6)
#     # driveBase.heading_control.pid(7558, 0, 1889, 0, 20)
#     driveBase.heading_control.pid(7558, 0, 1889, 1, 100)
#     driveBase.settings(speed_straight, acc_straight, speed_turn, acc_turn)
#     driveBase.straight(distance, stop, wait)

# initialize logging
log.loglevel=log.LOGLEVEL_INFO

# initialize hub
hub = InventorHub(top_side=Axis.Z, front_side=Axis.Y)

# initialize motors 
# motorArm = MotorAsServo(Port.F, Direction.COUNTERCLOCKWISE, -40, 215,name="MA")
motorColor = MotorAsServo(Port.C, Direction.CLOCKWISE, -150, 100,name="MC")

motorLeft = Motor(Port.A, reset_angle=None, )
motorRight = Motor(Port.B, Direction.COUNTERCLOCKWISE, reset_angle=None)

# # setting speed default values
# speed_straight = int(300)
# acc_straight  = int(100)
# speed_turn    = int(100)
# acc_turn      = int(50)

# driveBase = DriveBase(motorRight, motorLeft,56.009, 130.0 )
driveBase = myDriveBase(motorRight, motorLeft, 56.009, 130.0, 300, 100, 100, 50 )





sensorDistance = UltrasonicSensor(Port.D)
stopWatch = StopWatch()
sensorColor = ColorSensor(Port.E)


def setup():
    log.info("setup: starting application")

    hub.light.on(Color.GREEN)
    sensorDistance.lights.off()
    
    # motorArm.reset_angle()

def logAll():
    log.debug(f"{stopWatch.time():07d} US={sensorDistance.distance():04d} col={sensorColor.color()} L{motorLeft.angle():03d} R{motorRight.angle():03d} C{motorColor.motor.angle():03d} A{motorArm.motor.angle():03d}")

def logMotor():
    log.debug(f"L{motorLeft.angle():03d} R{motorRight.angle():03d} C{motorColor.motor.angle():03d}")

def logSensor():
    log.debug(f"{stopWatch.time():07d} US={sensorDistance.distance():04d} col={sensorColor.color()} colRefl={sensorColor.reflection()} colHSV={sensorColor.hsv()}")
    #  + " US=" + str (sensorDistance.distance()))

def stopOnColor(color: Color):
    # sensorColor.color(True)
    
    # log.debug(f" ref={sensorColor.reflection()} hsv={sensorColor.hsv(True)}")
    log.debug(f" col={sensorColor.color()}  ref={sensorColor.reflection()} hsv={sensorColor.hsv()}")



def main():
    wait(500)
    turnVar=-90
    hub.imu.reset_heading(0)
    headingTarget = 0
    wait (500)
    # motorLeft.run(-500)
    # motorRight.run(-500)
    driveBase.driveStraightToTarget(distance=1500,targetHeading=0,speed=900,acc=1000)
    
    wait(2000)
    log.info(f"{driveBase.done()} {driveBase.distance()}")
    # 
    # for i in range (30):
    #     log.debug(motorLeft.speed())
    #     wait(100)
    # log.debug("speedUp")
    # motorLeft.run(350)
    # for i in range (30):
    #     log.debug(motorLeft.speed())
    #     wait(100)
    # driveBase.driveStraightToTarget(-200, 0)
    # # motorLeft.run(100)
    # # driveBase.straight(300, wait=False)
    # # # while not driveBase.done():
    # while True:
    #      log.debug(f"t {headingTarget} l gimu {hub.imu.heading()} diff {diff} {driveBase.angle()} sl {motorLeft.speed()} sr {motorRight.speed()}" )
        
    #     logSensor()
    # #     if sensorColor.reflection() in  range(Constants.Color.Black.reflectionMin, Constants.Color.Black.reflectionMax):
    # #         driveBase.stop()

    # # while True:
    # #     stopOnColor(Color.BLACK)
    # # for rouds in range (2):
    # #     for turns in range (4): 
    # #         headingTarget = headingTarget + turnVar
    # #         turn(turnVar, wait=False)
    # #         while not driveBase.done():
    # #             wait (100)
    # #             currImu = hub.imu.heading()
    # #             diff = headingTarget - currImu
    # #             log.debug(f"t {headingTarget} l gimu {hub.imu.heading()} diff {diff} {driveBase.angle()} sl {motorLeft.speed()} sr {motorRight.speed()}" )
            
    # #         currImu = hub.imu.heading()
    # #         diff = headingTarget - currImu
    # #         if abs(diff) > 1:
    # #             print ("correction")
    # #             turn (diff)

    # #         driveBase.reset(angle=0)
    # #         driveStraight(300, wait=False, gyro=True)
    # #         while not driveBase.done():
    # #             lcurrImu = hub.imu.heading()
    # #             log.debug(f"t {headingTarget} l gimu {hub.imu.heading()} diff {diff} {driveBase.angle()} sl {motorLeft.speed()} sr {motorRight.speed()}" )
    # #             wait (100)

    # # motorLeft.reset_angle(0)
    # # motorRight.reset_angle(0)
    # # driveBase.reset(0,0)
    # # mleftStart = motorLeft.angle()
    # # mrightStart = motorRight.angle()
    # # mleftCurr = motorLeft.angle()
    # # mrightCurr = motorRight.angle()

    # # mleftDelta = mleftCurr - mleftStart
    # # mrightDelta = mrightCurr - mleftStart

    # # log.debug(f"l {mleftCurr:04d} r {mrightCurr:04d}  - dl {mleftDelta:04d} dr {mrightDelta:04d} -  gimu {hub.imu.heading()} {driveBase.angle()}" )

    # # driveBase.settings(speed_straight, acc_straight, speed_turn, acc_turn)    
    # # turn(turnVar, gyro=True, wait=False)
    # # # driveBase.use_gyro(False)
    # # # driveBase.arc(100, turnVar, None,Stop.COAST_SMART,False)


    # # while not driveBase.done():
    # #     mleftCurr = motorLeft.angle()
    # #     mrightCurr = motorRight.angle()

    # #     mleftDelta = mleftCurr - mleftStart
    # #     mrightDelta = mrightCurr - mleftStart
    # # log.debug(f"l {mleftCurr:04d} r {mrightCurr:04d}  - dl {mleftDelta:04d} dr {mrightDelta:04d} -  gimu {hub.imu.heading()} {driveBase.angle()}" )
        
    # # wait(1000)
    # # driveStraight(200, gyro=True, wait=True)
    # # while True:
    # #     log.debug(f"l {mleftCurr:04d} r {mrightCurr:04d}  - dl {mleftDelta:04d} dr {mrightDelta:04d} -  gimu {hub.imu.heading()} {driveBase.angle()}" )
    
    
    
    # # mleftEnd = motorLeft.angle()
    # # mrightEnd = motorRight.angle()


    # # for i in range(5):
    # #     log.debug(f"------ {i}")
    # #     driveStraight(1000)
    # #     turn(turnVar)
    # #     wait(100)
    # #     log.debug(hub.imu.heading())
    # #     driveStraight(200)
    # #     turn(turnVar)
    # #     wait(100)
    # #     log.debug(hub.imu.heading())
    # #     driveStraight(1000)
    # #     turn(turnVar)
    # #     wait(100)
    # #     log.debug(hub.imu.heading())
    # #     driveStraight(200)
    # #     turn(turnVar)
    # #     wait(100)
    # #     log.debug(hub.imu.heading())
    # # driveBase.straight(-1000)
    # # wait(1000)
    # motorLeft.control.target_tolerances(10,2)
    # motorRight.control.target_tolerances(10,2)
    # # # default (7558, 0, 1889, 3, 6)
    # # # driveBase.heading_control.pid(9000, 0, 50, 0.0, 20)
    # # # print(motorLeft.settings())
    # # # print(motorRight.settings())
    # # print (motorLeft.control.target_tolerances())
    # # print (motorRight.control.target_tolerances())
    # # motorStartLeft = motorLeft.angle()
    # # motorStartRight = motorRight.angle()
    
    # # driveBase.reset(0)
    
    # # speed=400
    # # acc=400
    # # driveBase.settings(speed,acc,50,50)
    # # driveBase.settings()

    # # driveBase.heading_control.target_tolerances(10,2)
    # # # driveBase.straight(3000,Stop.HOLD)
    # # driveBase.use_gyro(False)
    # # driveBase.turn(-360, Stop.HOLD, False)
    # # # driveBase.straight(50,Stop.HOLD)
    # # log.debug(hub.imu.heading())
    # # # motorEndLeft = motorLeft.angle()
    # # # motorEndRight = motorRight.angle()
    
    # # while not driveBase.done():
    # #     log.debug(hub.imu.heading())    
    # #     wait(100)  # Kurze Pause, um die CPU zu entlasten
    
    # # log.debug(hub.imu.heading())
    # # driveBase.use_gyro(True)
    # # driveBase.straight(1000,Stop.HOLD, False)
    # # while True:
    # #     log.debug(hub.imu.heading())
    # #     wait (100)
    # # # diffLeft=motorEndLeft - motorStartLeft
    # # # diffRight=motorEndRight - motorStartRight

    # # # log.info(f"deltaLeft = {diffLeft} deltaRight = {diffRight} speed = {speed:04d} acc= {acc:04d}")
    




    # # # while not (hub.imu.ready()):
    # # #     wait (5)
    # # # log.info("ready")

    # # # for x in range (40):
    # # #     motorRight.run_angle(1000, 417,Stop.HOLD,True)

    # # # while True:
    # # #     logAll()
    # #         # log.debug(hub.imu.heading())
    # #     # motorColor.moveToPosition(Constants.COLOR_SENSOR_DOWN,1000)
    # # # for x in range (40):
    # # #     log.debug(hub.imu.heading())
    # # #     # motorLeft.control.target_tolerances(10)
    # # #     motorLeft.control.limits(1000,1000)
    # # #     motorLeft.run_angle(1000, 417,Stop.HOLD,True)

    # # #     log.debug(hub.imu.heading())

    # # #     wait(500)



setup()
main()

