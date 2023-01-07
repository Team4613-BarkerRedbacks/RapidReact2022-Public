package redbacks.robot.subsystems.intersystem;

import redbacks.robot.subsystems.drivetrain.Drivetrain;
import redbacks.robot.subsystems.aiming.Aiming;

public class AimingToDrivetrainBindings {
    public static void bind(Aiming aiming, Drivetrain drivetrain) {
        aiming.getDrivetrainPositionSensor().populate(drivetrain::getPosition);
        aiming.getPositionOutput().attach(drivetrain::updatePositionFromVision);
        aiming.getDrivetrainState().populate(drivetrain::getHubRelativeChassisSpeeds);

        aiming.getHubRelativeYawDegrees().populate(drivetrain::getHubRelativeYawDegrees);
    }
}
