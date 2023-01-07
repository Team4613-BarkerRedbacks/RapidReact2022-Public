package redbacks.robot.subsystems.intersystem;

import redbacks.robot.subsystems.ballhandling.BallHandling;
import redbacks.robot.subsystems.drivetrain.Drivetrain;

public class BallHandlingToDrivetrainBindings {
    public static void bind(BallHandling ballHandling, Drivetrain drivetrain) {
        drivetrain.getHoldingOneBallSensor().populate(() -> ballHandling.isHoldingOneBall());
        drivetrain.getHoldingTwoBallsSensor().populate(() -> ballHandling.isFull());
    }
}
