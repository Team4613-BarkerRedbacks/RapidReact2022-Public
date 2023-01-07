package redbacks.robot.subsystems.climber;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;

import arachne.lib.io.GettableDouble;
import arachne.lib.scheduler.ScheduledBooleanSource;
import arachne.lib.scheduler.ScheduledSignal;
import redbacks.robot.Controllers;
import redbacks.robot.subsystems.climber.Climber.SystemState;

import static redbacks.robot.subsystems.climber.ClimberConstants.*;

public class ClimberBindings {
    public static void bind(Climber climber, ClimberHardware hardware, GettableDouble pitchDegrees, Controllers controllers) {
        bindControls(climber, controllers);
        bindInputs(climber, hardware, pitchDegrees);
        bindOutputs(climber, hardware);
    }

    private static void bindControls(Climber climber, Controllers controllers) {
        climber
        .addBinding(() -> climber.acceptManualMainHooksInput(-controllers.operator.getLeftY()));

        climber
        .addBinding(new ScheduledBooleanSource(false, controllers.operator::getXButton))
        .attach(climber::acceptManualRatchetUnlockInput);

        climber
        .addBinding(new ScheduledBooleanSource(false, () -> Controllers.isTriggerPressed(controllers.operator.getRightTriggerAxis())))
        .attach(climber::acceptManualExtendsSecondaryHooksInput);

        climber
        .addBinding(new ScheduledSignal(() -> controllers.operator.getAButton() && controllers.operator.getLeftBumper()))
        .attach(climber::raiseHooks);

        climber
        .addBinding(new ScheduledSignal(() -> controllers.operator.getBButton() && controllers.operator.getLeftBumper()))
        .attach(climber::climb);

        climber
        .addBinding(new ScheduledBooleanSource(false, () -> Controllers.isTriggerPressed(controllers.operator.getLeftTriggerAxis())))
        .attach((pressed) -> climber.setState(pressed ? SystemState.MANUAL : SystemState.PRE_CLIMB));

        climber
        .addBinding(new ScheduledBooleanSource(false, controllers.operator::getBackButton))
        .attach(climber::overridePitch);
    }

    private static void bindInputs(Climber climber, ClimberHardware hardware, GettableDouble pitchDegrees) {
        climber.getHookAtBottomSensor().populate(hardware.atBottomSensor);
        climber.getPositionSensor().populate(hardware.positionSensor);
        climber.getRobotPitchSensor().populate(pitchDegrees);
    }

    private static void bindOutputs(Climber climber, ClimberHardware hardware) {
        climber.getMainHooksPowerOutput().attach(hardware.mainHooks::set);
        climber.getMainHooksTargetPositionOutput().attach((metres) -> hardware.mainHooks.set(TalonFXControlMode.Position, metres / TICKS_TO_METRES));

        climber.getSecondaryHooksOutput().attach(hardware.secondaryHooksSolenoid::set);

        climber.getUnlockOutput().attach(hardware.unlockSolenoid::set);
        climber.getCoastModeOutput().attach((coast) ->  hardware.mainHooks.setNeutralMode(coast ? NeutralMode.Coast : NeutralMode.Brake));
    }
}
