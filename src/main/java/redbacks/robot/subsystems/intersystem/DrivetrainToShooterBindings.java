package redbacks.robot.subsystems.intersystem;

import redbacks.robot.subsystems.drivetrain.Drivetrain;
import redbacks.robot.subsystems.shooter.Shooter;

public class DrivetrainToShooterBindings {
    public static void bind(Drivetrain drivetrain, Shooter shooter) {
        shooter.getDistanceToHubMetresSensor().populate(drivetrain::getDistanceToHub);
    }
}
