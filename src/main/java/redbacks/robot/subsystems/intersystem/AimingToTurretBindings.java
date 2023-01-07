package redbacks.robot.subsystems.intersystem;

import redbacks.robot.subsystems.aiming.Aiming;
import redbacks.robot.subsystems.turret.Turret;

public class AimingToTurretBindings {
    public static void bind(Aiming aiming, Turret turret) {
        aiming.getTurretControlOutput().attach((control) -> turret.moveTurretToTarget(control.targetAngleDegrees, control.targetVelocityDegreesPerSec));
        aiming.getHoodAngleOutput().attach(turret::moveHoodToTarget);

        aiming.getTurretYawSensor().populate(turret::getYaw);
        aiming.getHoodPitchSensor().populate(turret::getPitch);
    }
}
