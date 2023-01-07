package redbacks.robot.subsystems.turret;

import arachne.lib.dashboard.Dashboard;
import arachne.lib.io.GettableDouble;
import arachne.lib.io.sensors.GettableDoubleInput;
import arachne.lib.io.sensors.Input;
import arachne.lib.logic.ArachneMath;
import arachne.lib.pipeline.DoublePipe;
import arachne.lib.pipeline.DoubleSource;
import arachne.lib.pipeline.Pipe;
import arachne.lib.pipeline.Source;
import arachne.lib.systems.Subsystem;

import static redbacks.robot.subsystems.turret.TurretConstants.*;

public class Turret extends Subsystem {
    private final Pipe<PositionAndFeedForward> targetYawAndFeedForward;
    private final DoublePipe targetPitch;

    private final GettableDoubleInput yaw;
    private final GettableDoubleInput pitch;
    private final GettableDoubleInput yawVelocityDegreesPerSec;

    private double hoodOffset = 0;

    public Turret() {
        targetYawAndFeedForward = new Pipe<PositionAndFeedForward>(new PositionAndFeedForward(0, 0));
        targetPitch = new DoublePipe((HOOD_MIN_ANGLE_DEGREES + HOOD_MAX_ANGLE_DEGREES) / 2);

        yaw = new GettableDoubleInput();
        pitch = new GettableDoubleInput();
        yawVelocityDegreesPerSec = new GettableDoubleInput();
    }

    @Override
    public void run() {
        super.run();
        Dashboard.getInstance().putNumber("Turret Yaw", yaw.get());
        Dashboard.getInstance().putNumber("Turret Pitch", pitch.get());
        Dashboard.getInstance().putNumber("Turret Velocity", yawVelocityDegreesPerSec.get());
        Dashboard.getInstance().putNumber("Turret Target Yaw", targetYawAndFeedForward.get().position);

        Dashboard.getInstance().putNumber("Hood Offset", hoodOffset);
    }

    public void moveTurretToTarget(double targetAngleDegrees, double targetVelocityDegreesPerSec) {
        if(targetAngleDegrees < WRAP_AROUND_POINT) {
            targetAngleDegrees = targetAngleDegrees + 360;
        }

        targetYawAndFeedForward.accept(new PositionAndFeedForward(
            ArachneMath.inBounds(targetAngleDegrees, TURRET_MIN_ANGLE_DEGREES, TURRET_MAX_ANGLE_DEGREES),
            targetVelocityDegreesPerSec * (1d / 700d)
        ));
    }

    public void moveHoodToTarget(double targetAngleDegrees) {
        Dashboard.getInstance().putNumber("Turret Target Pitch", targetAngleDegrees);
        targetPitch.accept(ArachneMath.inBounds(targetAngleDegrees + hoodOffset, HOOD_MIN_ANGLE_DEGREES, HOOD_MAX_ANGLE_DEGREES));
    }

    public double getYaw() {
        return yaw.get();
    }

    public double getPitch() {
        return pitch.get();
    }

    public Input<GettableDouble> getYawSensor() {
        return yaw;
    }

    public Input<GettableDouble> getPitchSensor() {
        return pitch;
    }

    public Input<GettableDouble> getYawVelocityDegreesPerSecSensor() {
        return yawVelocityDegreesPerSec;
    }

    public Source<PositionAndFeedForward> getTargetYawAndFeedForwardOutput() {
        return targetYawAndFeedForward;
    }

    public DoubleSource getTargetPitch() {
        return targetPitch;
    }

    public static class TurretControlInput {
        public final double targetAngleDegrees, targetVelocityDegreesPerSec;

        public TurretControlInput(double targetAngleDegrees, double targetVelocityDegreesPerSec) {
            this.targetAngleDegrees = targetAngleDegrees;
            this.targetVelocityDegreesPerSec = targetVelocityDegreesPerSec;
        }
    }

    static class PositionAndFeedForward {
        public final double position, feedForward;

        public PositionAndFeedForward(double position, double feedForward) {
            this.position = position;
            this.feedForward = feedForward;
        }
    }

    public void increaseHoodOffset() {
        hoodOffset += HOOD_OFFSET_INCREMENT_MPS;
    }

    public void decreaseHoodOffset() {
        hoodOffset -= HOOD_OFFSET_INCREMENT_MPS;
    }
}
