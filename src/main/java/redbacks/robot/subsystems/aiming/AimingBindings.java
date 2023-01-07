package redbacks.robot.subsystems.aiming;

import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import arachne.lib.scheduler.ScheduledBooleanSource;
import redbacks.robot.Controllers;
import redbacks.robot.subsystems.aiming.Aiming.SystemState;

import static redbacks.robot.subsystems.aiming.AimingConstants.*;

public class AimingBindings {
    public static void bind(Aiming aiming, AimingHardware hardware, Controllers controllers) {
        aiming
        .addBinding(() -> {
            PhotonPipelineResult result = hardware.limelight.getLatestResult();

            if(result.hasTargets()) {
                PhotonTrackedTarget target = result.getBestTarget();

                aiming.recalculatePosition(
                    Math.toRadians(-target.getYaw() - LIMELIGHT_YAW_OFFSET_DEGREES),
                    Math.toRadians(target.getPitch()) + LIMELIGHT_ANGLE_RADIANS,
                    result.getLatencyMillis() / 1000
                );
            }
        });

        aiming
        .addBinding(new ScheduledBooleanSource(false, controllers.driver::getLeftBumper))
        .attach((value) -> {
            if(value) aiming.setState(SystemState.OVERRIDE);
            else aiming.setState(SystemState.RUNNING);
        });
    }
}
