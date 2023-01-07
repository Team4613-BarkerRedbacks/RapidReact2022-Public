package redbacks.robot.subsystems.intersystem;

import redbacks.robot.subsystems.aiming.Aiming;
import redbacks.robot.subsystems.ballhandling.BallHandling;

public class AimingToBallHandlingBindings {
    public static void bind(Aiming aiming, BallHandling ballHandling) {
        aiming.getBallHandlingHasRightColourSensor().populate(ballHandling::hasRightBall);
    }
}
