package redbacks.robot.subsystems.drivetrain;

import arachne.lib.io.GettableDouble;
import arachne.lib.io.sensors.GettableDoubleInput;
import arachne.lib.io.sensors.Input;
import arachne.lib.pipeline.DoubleSource;
import arachne.lib.pipeline.DoublePipe;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveModule {
    // Outputs
    private final DoublePipe targetAngle, targetVelocity;

    // Sensors
    private final GettableDoubleInput angle, velocity;

    public SwerveModule() {
        this.targetAngle = new DoublePipe(0);
        this.targetVelocity = new DoublePipe(0);

        this.angle = new GettableDoubleInput();
        this.velocity = new GettableDoubleInput();
    }

    public void driveAt(SwerveModuleState targetState) {
        if(targetState.speedMetersPerSecond != 0) {
            SwerveModuleState optimisedState = optimize(targetState);

            if(optimisedState.speedMetersPerSecond != 0) targetAngle.accept(optimisedState.angle.getDegrees());
            targetVelocity.accept(optimisedState.speedMetersPerSecond);
        }
        else {
            targetVelocity.accept(0);
        }
    }

    private SwerveModuleState optimize(SwerveModuleState targetState) {
        double currentAngleDegrees = angle.get();
        double diff = targetState.angle.getDegrees() - currentAngleDegrees;

        double mod = diff >= 0 ? diff % 360 : 360 - (-diff % 360);

        if(mod <= 90) return new SwerveModuleState(targetState.speedMetersPerSecond, Rotation2d.fromDegrees(currentAngleDegrees + mod));
        else if(mod <= 270) return new SwerveModuleState(-targetState.speedMetersPerSecond, Rotation2d.fromDegrees(currentAngleDegrees + mod - 180));
        else return new SwerveModuleState(targetState.speedMetersPerSecond, Rotation2d.fromDegrees(currentAngleDegrees + mod - 360));
    }

    public SwerveModuleState getModuleState() {
        return new SwerveModuleState(velocity.get(), Rotation2d.fromDegrees(angle.get()));
    }

    public DoubleSource getTargetAngleDegreesOutput() {
        return targetAngle;
    }

    public DoubleSource getTargetVelocityMetresPerSecOutput() {
        return targetVelocity;
    }

    public Input<GettableDouble> getAngleSensor() {
        return angle;
    }

    public Input<GettableDouble> getVelocitySensor() {
        return velocity;
    }
}
