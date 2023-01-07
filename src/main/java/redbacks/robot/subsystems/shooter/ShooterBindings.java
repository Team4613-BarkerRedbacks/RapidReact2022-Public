package redbacks.robot.subsystems.shooter;

import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;

import arachne.lib.scheduler.ScheduledBooleanSource;
import arachne.lib.scheduler.ScheduledSignal;
import redbacks.robot.Controllers;

import static redbacks.robot.subsystems.shooter.ShooterConstants.*;

public class ShooterBindings {
    public static void bind(Shooter shooter, ShooterHardware hardware, Controllers controllers) {
		bindInputs(shooter, hardware);
		bindControls(shooter, controllers);
		bindOutputs(shooter, hardware);
    }

	private static void bindInputs(Shooter shooter, ShooterHardware hardware) {
		shooter.getVelocityMpsSensor().populate(() -> hardware.shooter.getSelectedSensorVelocity() * TICKS_PER_100MS_TO_METRES_PER_SECOND);
	}

	private static void bindControls(Shooter shooter, Controllers controllers) {
		shooter
		.addBinding(new ScheduledBooleanSource(false, () -> Controllers.isTriggerPressed(controllers.driver.getLeftTriggerAxis())))
		.attach(shooter::shootWhileMoving);

		shooter
		.addBinding(new ScheduledBooleanSource(false, () -> Controllers.isTriggerPressed(controllers.driver.getRightTriggerAxis())))
		.attach(shooter::shootStationary);

		shooter
		.addBinding(new ScheduledSignal(controllers.operator::getLeftStickButton))
		.attach(shooter::decreaseSpeedOffset);

		shooter
		.addBinding(new ScheduledSignal(controllers.operator::getRightStickButton))
		.attach(shooter::increaseSpeedOffset);

		shooter
		.addBinding(new ScheduledBooleanSource(false, controllers.driver::getRightBumper))
		.attach(shooter::lowGoalShoot);
	}

	private static void bindOutputs(Shooter shooter, ShooterHardware hardware) {
		shooter.getTargetVelocityMpsOutput().attach((mps) -> {
			if(mps == 0) hardware.shooter.set(TalonFXControlMode.PercentOutput, 0);
			else {
				double targetVelocityTicksPer100ms = mps / TICKS_PER_100MS_TO_METRES_PER_SECOND;
				hardware.shooter.set(TalonFXControlMode.Velocity,  targetVelocityTicksPer100ms, DemandType.ArbitraryFeedForward, hardware.getFeedForwardCoefficient() * targetVelocityTicksPer100ms);
				// hardware.shooter.set(TalonFXControlMode.PercentOutput, 0.8);
			}
		});
	}
}