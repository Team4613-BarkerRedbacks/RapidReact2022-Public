package redbacks.robot.subsystems.intersystem;

import redbacks.robot.subsystems.ballhandling.BallHandling;
import redbacks.robot.subsystems.shooter.Shooter;

public class BallHandlingToShooterBindings {
    public static void bind(BallHandling ballHandling, Shooter shooter) {
        shooter.getReadyToBeFedOutput().attach((activate) -> {
            if(activate) ballHandling.shoot();
            else ballHandling.intake();
        });

        shooter.getHasRightColourSensor().populate(ballHandling::hasRightBall);
    }
}
