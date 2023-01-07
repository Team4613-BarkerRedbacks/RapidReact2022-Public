package redbacks.robot.subsystems.drivetrain;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class DrivetrainConstants {
    static final double
        DISTANCE_BETWEEN_MODULE_LENGTH = 0.5853,
        DISTANCE_BETWEEN_MODULE_WIDTH = 0.5853,
        MODULE_DISTANCE_FROM_CENTRE_LENGTH = DISTANCE_BETWEEN_MODULE_LENGTH / 2,
        MODULE_DISTANCE_FROM_CENTRE_WIDTH = DISTANCE_BETWEEN_MODULE_WIDTH / 2,
        AUTO_MAX_ROTATIONAL_VELOCITY_RADIANS_PER_SEC = 3,
        TELEOP_MAX_ROTATIONAL_VELOCITY_RADIANS_PER_SEC = 8.978,
        MAX_VELOCITY_METRES_PER_SEC = 4.04,
        MAX_ACCELERATION_METRES_PER_SECOND_SQUARED = MAX_VELOCITY_METRES_PER_SEC / 1.4, // Accelerate up to speed in 1 second
        AUTO_MAX_ROTATIONAL_ACCELERATION_RADIANS_PER_SEC_SQUARED = AUTO_MAX_ROTATIONAL_VELOCITY_RADIANS_PER_SEC / 2; // Accelerate up to speed in 1 second

    static final double
        LINEAR_KP = 1.6,
        LINEAR_KI = 0.2,
        LINEAR_KD = 0.4,
        ROTATIONAL_KP = 9,
        ROTATIONAL_TRACKING_KP = 12,
        ROTATIONAL_KI = 0.7,
        ROTATIONAL_KD = 0.5;

    static final double
        POSITION_TOLERANCE_METRES = 0.15,
        ROTATION_TOLERANCE_RADIANS = Math.toRadians(5);

    static final TrapezoidProfile.Constraints
        LINEAR_MOTION_CONSTRAINTS = new TrapezoidProfile.Constraints(MAX_VELOCITY_METRES_PER_SEC, MAX_ACCELERATION_METRES_PER_SECOND_SQUARED),
        AUTO_ROTATION_MOTION_CONSTRAINTS = new TrapezoidProfile.Constraints(AUTO_MAX_ROTATIONAL_VELOCITY_RADIANS_PER_SEC, AUTO_MAX_ROTATIONAL_ACCELERATION_RADIANS_PER_SEC_SQUARED);

    public enum ModulePosition {
        // Positive x values represent moving toward the front of the robot whereas
        // positive y values represent moving toward the left of the robot.
        FRONT_LEFT(MODULE_DISTANCE_FROM_CENTRE_WIDTH, MODULE_DISTANCE_FROM_CENTRE_LENGTH),
        FRONT_RIGHT(MODULE_DISTANCE_FROM_CENTRE_WIDTH, -MODULE_DISTANCE_FROM_CENTRE_LENGTH),
        BACK_LEFT(-MODULE_DISTANCE_FROM_CENTRE_WIDTH, MODULE_DISTANCE_FROM_CENTRE_LENGTH),
        BACK_RIGHT(-MODULE_DISTANCE_FROM_CENTRE_WIDTH, -MODULE_DISTANCE_FROM_CENTRE_LENGTH);

        private final Translation2d offset;

        private ModulePosition(double xOffset, double yOffset) {
            this.offset = new Translation2d(xOffset, yOffset);
        }

        Translation2d getOffset() {
            return offset;
        }
    }

    // Deadzone Constants
	static final double
        DEADZONE_SIZE = 0.1,
        JOYSTICK_EXPONENT = 1.5;

    static double applyJoystickDeadzone(double value) {
        if(Math.abs(value) < DEADZONE_SIZE) return 0;

        if(value > 0) return (value - DEADZONE_SIZE) / (1 - DEADZONE_SIZE);
        else return (value + DEADZONE_SIZE) / (1 - DEADZONE_SIZE);
    }
}
