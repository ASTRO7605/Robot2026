import pygame
from typing import List, Self
import math
import bisect


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

    def plus(self, other: Self):
        return Angle(self.angleDeg + other.angleDeg)

    def unaryMinus(self):
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
        pygame.draw.circle(object, pygame.Color("#e9de0f"),
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
            pygame.draw.circle(self._object, pygame.Color("#e9de0f"),
                               (self._object.get_width() // 2,
                                self._object.get_height() // 2), 3)

        if self._active:
            drawVelocityArrow(self._master, self.position,
                              self._turretVelocity, pygame.Color("#c823dd"), 3)
            drawVelocityArrow(self._master, self.position, self._shotVelocity,
                              pygame.Color("#e9de0f"), 3)
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
    pygame.draw.circle(screen, pygame.Color("#c823dd"),
                       turretPose.tupleXYPixels(), 8)
    turretVelocity = getTurretVelocity()

    drawVelocityArrow(screen, turretPose, turretVelocity)


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
(shootOnMoveCorrection) = True
clock = pygame.time.Clock()
balls: List[Ball] = []
tofFromDistance = InterpolatingDoubleTreeMap()
distanceFromTof = InterpolatingDoubleTreeMap()


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
    # fix to use d / t instead of calculated speed
    estimatedDistance = targetDistance
    estimatedTof = tofFromDistance.get(estimatedDistance)

    if not shootOnMoveCorrection:
        shotSpeedVector = staticShot
    else:
        shotSpeedVector = staticShot.minus(turretVelocity)

    balls.append(
        Ball(screen, turretVelocity, shotSpeedVector, estimatedTof,
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

    keys = pygame.key.get_pressed()
    updateRobotSpeeds(keys, dt)

    hub.update(dt)
    robot.update(dt)
    for ball in balls:
        ball.update(dt)

    drawTurret(screen)

    pygame.display.flip()

pygame.quit()
