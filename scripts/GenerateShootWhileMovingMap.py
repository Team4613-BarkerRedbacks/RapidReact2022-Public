from math import acosh, atan, exp, log, sin, cos, sqrt, pi, degrees, radians, asin, nan, isnan

DELTA_HEIGHT_METRES = 1.78
GRAVITY_MPS2 = -9.8
BALL_MASS = 0.27
AIR_DENSITY = 1.225
BALL_RADIUS = 9.5 * 0.0254 / 2
BALL_AREA_CROSS_SECTION = pi * BALL_RADIUS ** 2
DRAG_COEFFICIENT = 0.5

HOOD_MIN_ANGLE_RADIANS = radians(53.14)
HOOD_MAX_ANGLE_RADIANS = radians(59.11)

FUDGE_FACTOR_SPEED_AT_5 = 7.6
FUDGE_FACTOR_SPEED_AT_7_5 = 10.5

XI = 50
YI = 50

def getHorizontalAngleRadians(robotVelocityYMps, exitVelocityMps, verticalAngleRadians):
    try:
        return asin(-robotVelocityYMps / (exitVelocityMps * cos(verticalAngleRadians)))
    except ValueError:
        return nan

def getDistanceToTargetMetres(exitVelocityMps, horizontalAngleRadians, verticalAngleRadians, robotVelocityXMps):
    k = (BALL_AREA_CROSS_SECTION * AIR_DENSITY * DRAG_COEFFICIENT) / (2 * BALL_MASS)
    vt = -sqrt(-GRAVITY_MPS2 / k)
    vt2 = vt ** 2
    uz = exitVelocityMps * sin(verticalAngleRadians)
    peak = - vt2 / (2 * GRAVITY_MPS2) * log(uz ** 2 / vt2 + 1)
    ez = exp(GRAVITY_MPS2 / vt2 * (DELTA_HEIGHT_METRES - peak))
    t = vt / GRAVITY_MPS2 * (acosh(ez) + atan(-uz / vt))
    ux = robotVelocityXMps + exitVelocityMps * cos(verticalAngleRadians) * cos(horizontalAngleRadians)
    sx = vt2 / GRAVITY_MPS2 * log(vt2 / (vt2 - ux * GRAVITY_MPS2 * t))

    return sx

def generateMap():
    data = {}

    for robotVelocityXmmps in range(-5000, 5000, XI):
        for robotVelocityYmmps in range(0, 5000, YI):
            for verticalAngleDegrees in map(lambda e: e / 10, range(640, 540 - 1, -5)):
                exitVelocityMps = float(f'{-0.1948 * verticalAngleDegrees + 17.643:.2f}')
                result = populateMap(data, exitVelocityMps, verticalAngleDegrees, robotVelocityXmmps, robotVelocityYmmps)

                if result is not None:
                    data[robotVelocityXmmps][robotVelocityYmmps] += result

    return data

def populateMap(data, exitVelocityMps, verticalAngleDegrees, robotVelocityXmmps, robotVelocityYmmps):
    fudgedExitVelocityMps = exitVelocityFudgeFactor(exitVelocityMps)
    fudgedVerticalAngleDegrees = verticalAngleDegrees + 3
    horizontalAngleDegrees = float(f'{degrees(getHorizontalAngleRadians(robotVelocityYmmps / 1000, fudgedExitVelocityMps * 1.4, radians(fudgedVerticalAngleDegrees))):.2f}')

    if not isnan(horizontalAngleDegrees):
        try:
            distanceToTargetMetres = float(f'{getDistanceToTargetMetres(fudgedExitVelocityMps, radians(horizontalAngleDegrees), radians(fudgedVerticalAngleDegrees), robotVelocityXmmps / 1000):.2f}')
            result = (distanceToTargetMetres, exitVelocityMps, horizontalAngleDegrees, verticalAngleDegrees)

            if distanceToTargetMetres > 0 and distanceToTargetMetres < 10:
                if robotVelocityXmmps not in data:
                    data[robotVelocityXmmps] = {}

                if robotVelocityYmmps not in data[robotVelocityXmmps]:
                    data[robotVelocityXmmps][robotVelocityYmmps] = ()

                return result

            return None

        except ValueError:
            return None

def exitVelocityFudgeFactor(speed):
    gradient = (FUDGE_FACTOR_SPEED_AT_7_5 - FUDGE_FACTOR_SPEED_AT_5) / (7.5 - 5)
    intercept = (-gradient * 5) + FUDGE_FACTOR_SPEED_AT_5
    
    return speed * gradient + intercept
    # return speed + 1 / (speed-5)
    # return speed

if __name__ == '__main__':
    with open('src/main/deploy/AimingMap.csv', 'w', encoding="utf-8") as f:
        data = generateMap()

        for k, v in data.items():
            output = str(k)
            for kk, vv in v.items():
                output += ";" + str(kk) + "," + ','.join(map(str, [*vv]))
            print(output, file = f)
