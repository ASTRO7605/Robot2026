import pygame
from typing import List, Self, Callable, Any
import math
import bisect

FRAMERATE = 60
HUB_DIAMETER = 1.067
BALL_DIAMETER = 0.15
FIELD_WIDTH = 8.069
ROBOT_LENGTH, ROBOT_WIDTH = (0.907, 0.823)
MAX_ROBOT_XY_SPEED, MAX_ROBOT_ANGULAR_SPEED = 4, 540
ROBOT_XY_ACCEL, ROBOT_ANGULAR_ACCEL = 2, 270

SCREEN_LENGTH, SCREEN_WIDTH = (1200, 800)
METER_TO_PIXEL = SCREEN_WIDTH / FIELD_WIDTH
MAX_ARROW_LENGTH = 100
SPEED_TO_PIXEL = (MAX_ARROW_LENGTH / MAX_ROBOT_XY_SPEED) / METER_TO_PIXEL

TURRET_COLOR = pygame.Color("#c823dd")
SHOT_COLOR = pygame.Color("#e9de0f")


class InterpolatingDoubleTreeMap:
    """
    A map that stores double keys and values, allowing linear interpolation
    for keys not exactly present. Similar to WPILib's InterpolatingTreeMap.
    """

    def __init__(self):
        self._data = [
        ]  # List of (key: float, value: float) tuples, sorted by key

    def put(self, key: float, value: float):
        """Insert or update a key-value pair."""
        # Find insertion point
        idx = bisect.bisect_left(self._data, (key, float('-inf')))
        if idx < len(self._data) and self._data[idx][0] == key:
            # Update existing
            self._data[idx] = (key, value)
        else:
            # Insert new
            self._data.insert(idx, (key, value))

    def get(self, key: float) -> float:
        """Get the interpolated value for the given key."""
        if not self._data:
            raise ValueError("Map is empty")

        # Find the largest index where data[idx][0] <= key
        idx = bisect.bisect_right(self._data, (key, float('inf'))) - 1

        if idx < 0:
            # Key is less than all keys, return first value
            return self._data[0][1]
        elif idx == len(self._data) - 1:
            # Key is greater than or equal to last key, return last value
            return self._data[-1][1]
        else:
            # Interpolate between idx and idx+1
            k1, v1 = self._data[idx]
            k2, v2 = self._data[idx + 1]
            if k2 == k1:
                return v1
            # Linear interpolation
            return v1 + (v2 - v1) * (key - k1) / (k2 - k1)

    def clear(self):
        """Clear all entries."""
        self._data.clear()

    def __len__(self) -> int:
        return len(self._data)


class PygameButton:

    def __init__(self,
                 master: pygame.Surface,
                 centerPos: tuple[int, int],
                 size: tuple[int, int],
                 cmd: Callable[[], Any] = lambda: None,
                 bgColor: str | tuple[int, int, int] = "#BBBBBB",
                 hoverColor: str | tuple[int, int, int] = "#AAAAAA",
                 text: pygame.Surface | None = None,
                 polygon: list[tuple[int, int]] | None = None,
                 polygonColor: str | tuple[int, int, int] = "white",
                 soundFile: str = ""):
        self.master = master
        self.rect = pygame.Rect(centerPos, size)
        self.rect.center = centerPos
        self.cmd = cmd
        self.bgColor = pygame.Color(bgColor)
        self.hoverColor = pygame.Color(hoverColor)
        self.text = text
        self.polygon = polygon
        self.polygonColor = polygonColor
        self.sound = pygame.mixer.Sound(soundFile) if soundFile else None
        self.actif = False

    def update(self):
        self.actif = True
        # changer la couleur si le curseur est dans le bouton
        couleur = self.hoverColor if self.rect.collidepoint(
            pygame.mouse.get_pos()) else self.bgColor
        pygame.draw.rect(self.master,
                         couleur,
                         self.rect,
                         border_radius=self.rect.height // 5)

        # rajouter le texte dans le bouton s'il y en a
        if self.text:
            self.master.blit(self.text,
                             self.text.get_rect(center=self.rect.center))
        # dessiner la forme sur le bouton s'il y en a une
        if self.polygon:
            pygame.draw.polygon(self.master, self.polygonColor, self.polygon)

    def desactive(self):
        self.actif = False

    def checkClick(self):
        if not self.actif:
            return
        # si clic dans le bouton, appeler la commande et jouer le son
        if self.rect.collidepoint(pygame.mouse.get_pos()):
            if self.sound: self.sound.play()
            self.cmd()

    # changer le texte du bouton
    def changeText(self, text: pygame.Surface):
        self.text = text

    def setCmd(self, cmd: Callable[[], Any]):
        self.cmd = cmd

    def getText(self):
        return self.text


class Angle:

    def __init__(self, angleDeg: float):
        self._angleDeg = angleDeg

    @property
    def angleDeg(self):
        return self._angleDeg

    @angleDeg.setter
    def angleDeg(self, newAngleDeg: float):
        self._angleDeg = newAngleDeg

    @property
    def angleRad(self):
        return self.angleDeg * (math.pi / 180)

    @angleRad.setter
    def angleRad(self, newAngleRad: float):
        self.angleDeg = newAngleRad * (180 / math.pi)

    def plus(self, other: "Angle") -> "Angle":
        return Angle(self.angleDeg + other.angleDeg)

    def unaryMinus(self) -> "Angle":
        return Angle(-self.angleDeg)


class Position:

    def __init__(self, x: float, y: float, angle: Angle):
        self._x = x
        self._y = y
        self._angle = angle

    @property
    def x(self):
        return self._x

    @x.setter
    def x(self, newX: float):
        self._x = newX

    @property
    def xPixels(self):
        return self._x * METER_TO_PIXEL

    @property
    def yPixels(self):
        return SCREEN_WIDTH - self._y * METER_TO_PIXEL

    @property
    def y(self):
        return self._y

    @y.setter
    def y(self, newY: float):
        self._y = newY

    @property
    def angle(self):
        return self._angle

    @angle.setter
    def angle(self, newAngle: Angle):
        self._angle = newAngle

    def tupleXYPixels(self, maxY: float = SCREEN_WIDTH):
        return (self.x * METER_TO_PIXEL, maxY - self.y * METER_TO_PIXEL)

    def rotateBy(self, angle: Angle):
        return Position(
            self.x * math.cos(angle.angleRad) -
            self.y * math.sin(angle.angleRad),
            self.y * math.cos(angle.angleRad) +
            self.x * math.sin(angle.angleRad), self.angle.plus(angle))

    def plus(self, other: Self):
        return Position(self.x + other.x, self.y + other.y,
                        self.angle.plus(other.angle))

    def unaryMinus(self) -> Self:
        return Position(-self.x, -self.y,
                        self.angle.unaryMinus())  # type: ignore

    def minus(self, other: Self):
        other = other.unaryMinus()
        return self.plus(other)

    def getXYAngle(self) -> Angle:
        return Angle(
            math.atan2(self.y, self.x if self.x != 0 else 0.00001) *
            (180 / math.pi))

    def getXYNorm(self) -> float:
        return (self.x**2 + self.y**2)**0.5


class Velocity:

    def __init__(self, vx: float, vy: float, vtheta: float):
        self._vx = vx
        self._vy = vy
        self._vtheta = vtheta

    @property
    def vx(self):
        return self._vx

    @vx.setter
    def vx(self, newX: float):
        self._vx = newX

    @property
    def vy(self):
        return self._vy

    @vy.setter
    def vy(self, newY: float):
        self._vy = newY

    @property
    def vtheta(self):
        return self._vtheta

    @vtheta.setter
    def vtheta(self, newVtheta: float):
        self._vtheta = newVtheta

    def getXYNorm(self) -> float:
        return (self.vx**2 + self.vy**2)**0.5

    def getXYAngle(self) -> Angle:
        return Angle(
            math.atan2(self.vy, self.vx if self.vx != 0 else 0.00001) *
            (180 / math.pi))

    def copy(self) -> Self:
        return Velocity(self.vx, self.vy, self.vtheta)  # type: ignore

    def plus(self, other: Self) -> Self:
        return Velocity(self.vx + other.vx, self.vy + other.vy,
                        self.vtheta + other.vtheta)  # type: ignore

    def unaryMinus(self) -> Self:
        return Velocity(-self.vx, -self.vy, -self.vtheta)  # type: ignore

    def minus(self, other) -> Self:
        return self.plus(other.unaryMinus())

    def setXYNorm(self, newNorm: float):
        self.vx = newNorm * math.cos(self.getXYAngle().angleRad)
        self.vy = newNorm * math.sin(self.getXYAngle().angleRad)


class CanvasObject:

    def __init__(self,
                 master: pygame.Surface,
                 object: pygame.Surface,
                 coordinates=Position(0, 0, Angle(0)),
                 velocity=Velocity(0, 0, 0)):
        self._master = master
        self._object = object
        self._coords = coordinates
        self._velocity = velocity.copy()

    @property
    def velocity(self):
        return self._velocity

    @velocity.setter
    def velocity(self, newVel: Velocity):
        self._velocity = newVel

    @property
    def position(self):
        return self._coords

    @position.setter
    def position(self, newPos: Position):
        self._coords = newPos

    def update(self, dt: float):
        self.position.x += self.velocity.vx * dt
        self.position.y += self.velocity.vy * dt
        self.position.angle.angleDeg += self.velocity.vtheta * dt

        rotated = pygame.transform.rotate(self._object,
                                          self.position.angle.angleDeg)
        self._master.blit(
            rotated, rotated.get_rect(center=self.position.tupleXYPixels()))


class Ball(CanvasObject):

    def __init__(self,
                 master: pygame.Surface,
                 turretVelocity: Velocity,
                 shotVelocity: Velocity,
                 tof: float,
                 coordinates=Position(0, 0, Angle(0))):
        self._turretVelocity = turretVelocity
        self._shotVelocity = shotVelocity
        self._tof = tof

        self._active = True

        object = pygame.Surface(
            (BALL_DIAMETER * METER_TO_PIXEL, BALL_DIAMETER * METER_TO_PIXEL),
            pygame.SRCALPHA)
        object.fill(pygame.Color(0, 0, 0, 0))
        pygame.draw.circle(object, SHOT_COLOR,
                           (object.get_width() // 2, object.get_height() // 2),
                           BALL_DIAMETER * METER_TO_PIXEL // 2)
        super().__init__(master, object, coordinates,
                         turretVelocity.plus(shotVelocity))

    def update(self, dt: float):
        super().update(dt)
        self._tof -= dt
        if self._tof <= 0 and self._active:
            self._active = False
            self._velocity = Velocity(0, 0, 0)
            self._object.fill((0, 0, 0, 0))
            pygame.draw.circle(self._object, SHOT_COLOR,
                               (self._object.get_width() // 2,
                                self._object.get_height() // 2), 3)

        if self._active:
            drawVelocityArrow(self._master, self.position,
                              self._turretVelocity, TURRET_COLOR, 3)
            drawVelocityArrow(self._master, self.position, self._shotVelocity,
                              SHOT_COLOR, 3)
            drawVelocityArrow(self._master,
                              self.position,
                              self.velocity,
                              lineWidth=3)


def drawArrow(surface: pygame.Surface,
              start: Position,
              end: Position,
              color: pygame.Color = pygame.Color(0, 0, 0),
              line_width: int = 5,
              arrowhead_length: float = 15,
              arrow_width: float = 16):
    # Calculate direction
    dx = end.xPixels - start.xPixels
    dy = end.yPixels - start.yPixels
    length = math.sqrt(dx**2 + dy**2)
    if length < arrowhead_length or length == 0:
        return
    unit_x = dx / length
    unit_y = dy / length
    # Arrowhead base point (shaft end)
    arrow_x = end.xPixels - arrowhead_length * unit_x
    arrow_y = end.yPixels - arrowhead_length * unit_y

    # Draw line from start to arrowhead base (not through tip)
    pygame.draw.line(surface, color, start.tupleXYPixels(), (arrow_x, arrow_y),
                     line_width)

    # Arrowhead points
    perp_x = -unit_y
    perp_y = unit_x
    half_width = arrow_width / 2
    left_x = arrow_x + half_width * perp_x
    left_y = arrow_y + half_width * perp_y
    right_x = arrow_x - half_width * perp_x
    right_y = arrow_y - half_width * perp_y
    pygame.draw.polygon(
        surface, color,
        [end.tupleXYPixels(), (left_x, left_y), (right_x, right_y)])


def accelToTarget(current: float, target: float, accel: float,
                  dt: float) -> float:
    newValue = current
    if current < target:
        newValue = current + accel * dt
        if newValue > target:
            newValue = target
    elif current > target:
        newValue = current - accel * dt
        if newValue < target:
            newValue = target

    return newValue


def updateRobotSpeeds(keys: pygame.key.ScancodeWrapper, dt: float):
    # xy movement
    xInput = int(keys[pygame.K_d]) - int(keys[pygame.K_a])
    yInput = int(keys[pygame.K_w]) - int(keys[pygame.K_s])
    direction = math.atan2(yInput, xInput if xInput != 0 else 0.00001)

    xInput = xInput * abs(math.cos(direction))
    yInput = yInput * abs(math.sin(direction))

    xCommanded = MAX_ROBOT_XY_SPEED * xInput
    yCommanded = MAX_ROBOT_XY_SPEED * yInput

    robot.velocity.vx = accelToTarget(robot.velocity.vx, xCommanded,
                                      MAX_ROBOT_XY_SPEED, dt)
    robot.velocity.vy = accelToTarget(robot.velocity.vy, yCommanded,
                                      MAX_ROBOT_XY_SPEED, dt)

    # rotation movement
    rotation = int(keys[pygame.K_LEFT]) - int(keys[pygame.K_RIGHT])

    rotationCommanded = MAX_ROBOT_ANGULAR_SPEED * rotation

    robot.velocity.vtheta = accelToTarget(robot.velocity.vtheta,
                                          rotationCommanded,
                                          MAX_ROBOT_ANGULAR_SPEED, dt)


def getTurretVelocity() -> Velocity:
    turretVelocity = Velocity(robot.velocity.vx, robot.velocity.vy, 0)
    turretDistanceFromCenter = TURRET_IN_ROBOT_POS.rotateBy(
        robot.position.angle)

    turretVelocity.vx -= math.radians(
        robot.velocity.vtheta) * turretDistanceFromCenter.y
    turretVelocity.vy += math.radians(
        robot.velocity.vtheta) * turretDistanceFromCenter.x

    return turretVelocity


def drawVelocityArrow(screen: pygame.Surface,
                      startPose: Position,
                      velocity: Velocity,
                      color: pygame.Color = pygame.Color(0, 0, 0),
                      lineWidth: int = 5,
                      headLength: int = 15,
                      headWidth: int = 15):
    endArrow = Position(startPose.x + velocity.vx * SPEED_TO_PIXEL,
                        startPose.y + velocity.vy * SPEED_TO_PIXEL, Angle(0))
    drawArrow(screen, startPose, endArrow, color, lineWidth, headLength,
              headWidth)


def drawTurret(screen: pygame.Surface):
    turretPose = robot.position.plus(
        TURRET_IN_ROBOT_POS.rotateBy(robot.position.angle))
    pygame.draw.circle(screen, TURRET_COLOR, turretPose.tupleXYPixels(), 8)
    turretVelocity = getTurretVelocity()

    drawVelocityArrow(screen, turretPose, turretVelocity, TURRET_COLOR)


pygame.init()

screen = pygame.display.set_mode((SCREEN_LENGTH, SCREEN_WIDTH))

hubSurface = pygame.Surface(
    (HUB_DIAMETER * METER_TO_PIXEL, HUB_DIAMETER * METER_TO_PIXEL),
    pygame.SRCALPHA)
hubSurface.fill(pygame.Color(0, 0, 0, 0))
pygame.draw.circle(hubSurface, pygame.Color("#3D3D3D"),
                   (hubSurface.get_width() // 2, hubSurface.get_height() // 2),
                   HUB_DIAMETER * METER_TO_PIXEL // 2, 5)
hub = CanvasObject(
    screen, hubSurface,
    Position((screen.get_width() // 2) / METER_TO_PIXEL,
             (screen.get_height() // 2) / METER_TO_PIXEL, Angle(0)))

TURRET_IN_ROBOT_POS = Position(0.184, 0, Angle(0))

robotSurface = pygame.Surface(
    (ROBOT_LENGTH * METER_TO_PIXEL, ROBOT_WIDTH * METER_TO_PIXEL),
    pygame.SRCALPHA)
robotSurface.fill(pygame.Color(0, 0, 0, 0))
pygame.draw.rect(
    robotSurface, pygame.Color(8, 11, 189),
    (0, 0, ROBOT_LENGTH * METER_TO_PIXEL, ROBOT_WIDTH * METER_TO_PIXEL), 10)
robot = CanvasObject(screen, robotSurface, Position(0.5, 0.5, Angle(0)))

running = True
shootOnMoveCorrection = True
distanceCorrection = True
clock = pygame.time.Clock()
balls: List[Ball] = []
tofFromDistance = InterpolatingDoubleTreeMap()
distanceFromTof = InterpolatingDoubleTreeMap()

correctionBtn = PygameButton(screen, (SCREEN_LENGTH - 100, 150), (100, 50),
                             text=pygame.font.SysFont(None, 15).render(
                                 "CORRECTION", True, (0, 0, 0)),
                             bgColor="#15DD1F",
                             hoverColor="#5DEF64")

resultingText = pygame.font.SysFont(None, 20).render("RESULTING VELOCITY",
                                                     True, (0, 0, 0))
shotText = pygame.font.SysFont(None, 20).render("SHOT VELOCITY", True,
                                                (0, 0, 0))
turretText = pygame.font.SysFont(None, 20).render("TURRET VELOCITY", True,
                                                  (0, 0, 0))


def switchCorrectionMode():
    global shootOnMoveCorrection
    shootOnMoveCorrection = not shootOnMoveCorrection
    if shootOnMoveCorrection:
        correctionBtn.bgColor = pygame.Color("#15DD1F")
        correctionBtn.hoverColor = pygame.Color("#5DEF64")

    else:
        correctionBtn.bgColor = pygame.Color("#DD1515")
        correctionBtn.hoverColor = pygame.Color("#E15454")


correctionBtn.setCmd(switchCorrectionMode)


def fillTables(distance: float, tof: float):
    tofFromDistance.put(distance, tof)
    distanceFromTof.put(tof, distance)


fillTables(1.38, 0.9)
fillTables(1.88, 1.09)
fillTables(3.15, 1.11)
fillTables(4.55, 1.12)
fillTables(5.68, 1.16)


def spawnBall(screen: pygame.Surface):
    turretVelocity = getTurretVelocity()
    turretPose = robot.position.plus(
        TURRET_IN_ROBOT_POS.rotateBy(robot.position.angle))

    turretToTarget = hub.position.minus(turretPose)
    targetDistance = turretToTarget.getXYNorm()

    staticSpeed = targetDistance / tofFromDistance.get(targetDistance)
    staticShot = Velocity(
        staticSpeed * math.cos(turretToTarget.getXYAngle().angleRad),
        staticSpeed * math.sin(turretToTarget.getXYAngle().angleRad), 0)

    if not shootOnMoveCorrection:
        shotVelocityVector = staticShot
    else:
        shotVelocityVector = staticShot.minus(turretVelocity)

    estimatedDistance = targetDistance
    estimatedTof = tofFromDistance.get(estimatedDistance)
    resultingSpeed = estimatedDistance / estimatedTof

    if distanceCorrection:
        estimatedTof = tofFromDistance.get(estimatedDistance)
        resultingSpeed = estimatedDistance / estimatedTof

        ratio = shotVelocityVector.getXYNorm() / resultingSpeed
        estimatedDistance *= ratio

    shotVelocityVector.setXYNorm(estimatedDistance / estimatedTof)

    balls.append(
        Ball(screen, turretVelocity, shotVelocityVector, estimatedTof,
             turretPose))


while running:
    dt = clock.tick(FRAMERATE) / 1000

    screen.fill("light grey")

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_SPACE:
                spawnBall(screen)
        if event.type == pygame.MOUSEBUTTONDOWN:
            correctionBtn.checkClick()

    keys = pygame.key.get_pressed()
    updateRobotSpeeds(keys, dt)

    hub.update(dt)
    robot.update(dt)
    for ball in balls:
        ball.update(dt)

    drawTurret(screen)

    correctionBtn.update()
    screen.blit(resultingText,
                resultingText.get_rect(midright=(SCREEN_LENGTH - 150, 50)))
    drawArrow(
        screen,
        Position((SCREEN_LENGTH - 125) / METER_TO_PIXEL,
                 (SCREEN_WIDTH - 50) / METER_TO_PIXEL, Angle(0)),
        Position((SCREEN_LENGTH - 50) / METER_TO_PIXEL,
                 (SCREEN_WIDTH - 50) / METER_TO_PIXEL, Angle(0)))

    screen.blit(shotText,
                shotText.get_rect(midright=(SCREEN_LENGTH - 150, 75)))
    drawArrow(
        screen,
        Position((SCREEN_LENGTH - 125) / METER_TO_PIXEL,
                 (SCREEN_WIDTH - 75) / METER_TO_PIXEL, Angle(0)),
        Position((SCREEN_LENGTH - 50) / METER_TO_PIXEL,
                 (SCREEN_WIDTH - 75) / METER_TO_PIXEL, Angle(0)), SHOT_COLOR)

    screen.blit(turretText,
                turretText.get_rect(midright=(SCREEN_LENGTH - 150, 100)))
    drawArrow(
        screen,
        Position((SCREEN_LENGTH - 125) / METER_TO_PIXEL,
                 (SCREEN_WIDTH - 100) / METER_TO_PIXEL, Angle(0)),
        Position((SCREEN_LENGTH - 50) / METER_TO_PIXEL,
                 (SCREEN_WIDTH - 100) / METER_TO_PIXEL, Angle(0)),
        TURRET_COLOR)

    pygame.display.flip()

pygame.quit()
