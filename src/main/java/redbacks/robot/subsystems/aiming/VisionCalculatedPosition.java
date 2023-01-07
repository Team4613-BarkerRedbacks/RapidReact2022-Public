package redbacks.robot.subsystems.aiming;

import edu.wpi.first.math.geometry.Pose2d;

public class VisionCalculatedPosition {
    public final Pose2d position;
    public final double latencySeconds;

    public VisionCalculatedPosition(Pose2d position, double latencySeconds) {
        this.position = position;
        this.latencySeconds = latencySeconds;
    }
}
