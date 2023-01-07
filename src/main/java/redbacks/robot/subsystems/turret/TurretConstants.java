package redbacks.robot.subsystems.turret;

import redbacks.robot.Constants;

public class TurretConstants {
    static final double
        // Turret
        TURRET_MAX_ANGLE_DEGREES = 225,
        TURRET_MIN_ANGLE_DEGREES = -90,
        WRAP_AROUND_POINT = (TURRET_MAX_ANGLE_DEGREES + TURRET_MIN_ANGLE_DEGREES - 360) / 2,

        TURRET_KP = 0.4,
        TURRET_KI = 0,
        TURRET_KD = 0,
        TURRET_KF = Constants.TALON_FX_VELOCITY_MAX_OUTPUT * 0.4 / 7800,
        TURRET_CRUISE_VELOCITY_DEGREES_PER_SEC = 400,
        TURRET_MAX_ACCELERATION_DEGREES_PER_SEC_SQUARED = 1200,
        // TURRET_PROFILE_SMOOTHNESS = 1,
        // TURRET_MAX_PID_OUTPUT = 0.8,

        TURRET_TICKS_TO_DEGREES = 360d / Constants.TALON_FX_TICKS_PER_REVOLUTION * 16 / 94 * 12 / 150,

        // Hood
        HOOD_MAX_ANGLE_DEGREES = 64,// Calculated from 90 - 25.04,
        HOOD_MIN_ANGLE_DEGREES = 54,// Calculated from 90 - 36.43,
        HOOD_ABSOLUTE_READING_AT_BASE = 12,

        HOOD_TICKS_TO_DEGREES = 360d / Constants.MAG_ENCODER_TICKS_PER_REVOLUTION * 18 / 456,

        HOOD_KP = 0.5,
        HOOD_KI = 0,
        HOOD_KD = 0,
        HOOD_KF = Constants.TALON_FX_VELOCITY_MAX_OUTPUT * 0.8 / 350,
        HOOD_CRUISE_VELOCITY_DEGREES_PER_SEC = 8,
        HOOD_MAX_ACCELERATION_DEGREES_PER_SEC_SQUARED = 32,

        HOOD_OFFSET_INCREMENT_MPS = 0.5;

    public static boolean isAngleInRange(double angle) {
        while(angle < 0) angle += 360;

        return angle < TURRET_MAX_ANGLE_DEGREES || angle > TURRET_MIN_ANGLE_DEGREES + 360;
    }
}