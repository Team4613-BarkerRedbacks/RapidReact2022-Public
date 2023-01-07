package redbacks.robot.subsystems.ballhandling;

import arachne.lib.scheduler.ScheduledBooleanSource;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import redbacks.robot.Controllers;
import static redbacks.robot.subsystems.ballhandling.BallHandlingConstants.*;
public class BallHandlingBindings {
    public static void bind(BallHandling ballHandling, BallHandlingHardware hardware, Controllers controllers) {
        bindControls(ballHandling, controllers);
        bindOutputs(ballHandling, hardware);
        bindInputs(ballHandling, hardware);
    }

    private static void bindControls(BallHandling ballHandling, Controllers controllers) {
        ballHandling
        .addBinding(new ScheduledBooleanSource(false, controllers.operator::getYButton))
        .attach((activate) -> {
            if(activate) ballHandling.outtake();
            else ballHandling.intake();
        });

        ballHandling
        .addBinding(new ScheduledBooleanSource(false, controllers.operator::getRightBumper))
        .attach(ballHandling::intakeManually);

        ballHandling.getManuallyRetractIntake().populate(() -> controllers.operator.getPOV() != -1);

        ballHandling
        .getRumbleControllerOutput()
        .attach((shouldRumble) -> controllers.driver.setRumble(RumbleType.kLeftRumble, shouldRumble ? RUMBLE_STRENGTH : 0));
    }

    private static void bindOutputs(BallHandling ballHandling, BallHandlingHardware hardware) {
        ballHandling.getIntakeOutput().attach(hardware.intake::set);
        ballHandling.getTowerOutput().attach(hardware.tower::set);

        ballHandling.getExtendIntakeOutput().attach(hardware.intakeExtension::set);
    }

    private static void bindInputs(BallHandling ballHandling, BallHandlingHardware hardware) {
        ballHandling.getBallAtBottomSensor().populate(hardware.ballAtBottom);
        ballHandling.getBallAtTopSensor().populate(hardware.ballAtTop);

        ballHandling.getPositionTicksSensor().populate(hardware.towerPositionTicks);

        ballHandling.getBallColourSensor().populate(hardware.currentColour);
    }
}
