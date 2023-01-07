package redbacks.robot.subsystems.drivetrain;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;

import static redbacks.robot.subsystems.drivetrain.SwerveModuleConstants.*;

public class SwerveModuleBindings {
    public static void bind(SwerveModule module, SwerveModuleHardware hardware) {
        module.getAngleSensor().populate(hardware.angle);
        module.getVelocitySensor().populate(hardware.velocity);

        module.getTargetAngleDegreesOutput().attach((degrees) -> hardware.steer.set(TalonFXControlMode.Position, degrees / STEER_TICKS_TO_DEGREES));

        module.getTargetVelocityMetresPerSecOutput().attach((metresPerSec) -> hardware.drive.set(TalonFXControlMode.Velocity,
            metresPerSec / DrivetrainConstants.MAX_VELOCITY_METRES_PER_SEC * MAX_WHEEL_ENCODER_VELOCITY_PER_100MS)
        );
    }
}