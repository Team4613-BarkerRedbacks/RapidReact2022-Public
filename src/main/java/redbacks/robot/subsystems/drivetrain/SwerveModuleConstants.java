package redbacks.robot.subsystems.drivetrain;

import java.security.InvalidParameterException;

import redbacks.robot.Constants;
import redbacks.robot.subsystems.drivetrain.DrivetrainConstants.ModulePosition;

public class SwerveModuleConstants {
    private static final double
        WHEEL_RADIUS = 0.05,
        WHEEL_CIRCUMFERENCE = 2 * Math.PI * WHEEL_RADIUS,
        SHAFT_TO_WHEEL_RATIO = 8.14, // Retrieved from documented module gear ratio
        ENCODER_COUNTS_PER_WHEEL_ROTATION = Constants.TALON_FX_TICKS_PER_REVOLUTION * SHAFT_TO_WHEEL_RATIO,
        SECONDS_FROM_100MS_MULTIPLIER = 10,
        SHAFT_TO_STEER_RATIO = 12.8;

	static final double
        MAX_WHEEL_ENCODER_VELOCITY_PER_100MS = 21900, // TODO Get a non-freespin value
        ENCODER_VELOCITY_TO_METRES_PER_SECOND_MULTIPLIER = (WHEEL_CIRCUMFERENCE * SECONDS_FROM_100MS_MULTIPLIER) / ENCODER_COUNTS_PER_WHEEL_ROTATION,
        STEER_TICKS_TO_DEGREES = 360 / Constants.TALON_FX_TICKS_PER_REVOLUTION / SHAFT_TO_STEER_RATIO;

    static final double
        STEER_KP = 0.25,
        STEER_KI = 0,
        STEER_KD = 0;

    static final double
        VELOCITY_KP = 2.4e-5,
        VELOCITY_KI = 0,
        VELOCITY_KD = 3.5e-4,
        VELOCITY_KF = 0.0467,
        VELOCITY_MAX_PID_OUTPUT = 1;

    static double ANGLE_OFFSET_FOR(ModulePosition position) {
        switch(position) {
            case BACK_LEFT: return -35.9;
            case BACK_RIGHT: return -10;
            case FRONT_LEFT: return 74.1;
            case FRONT_RIGHT: return -162.7;
            default: throw new InvalidParameterException("Invalid module position");
        }
    }
}
