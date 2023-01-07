package redbacks.robot.subsystems.intersystem;

import redbacks.robot.subsystems.aiming.Aiming;
import redbacks.robot.subsystems.shooter.Shooter;

public class AimingToShooterBindings {
    public static void bind(Aiming aiming, Shooter shooter) {
        aiming.getIsStrategyShootWhileMovingSensor().populate(shooter::isStrategyShootWhileMoving);
	    aiming.getShootingSpeedOutput().attach(shooter::acceptShootingValues);
    }
}
