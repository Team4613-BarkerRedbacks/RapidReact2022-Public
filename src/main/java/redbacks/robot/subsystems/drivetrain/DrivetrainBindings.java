package redbacks.robot.subsystems.drivetrain;

import static redbacks.robot.subsystems.drivetrain.DrivetrainConstants.*;

import arachne.lib.logic.ArachneMath;
import arachne.lib.scheduler.ScheduledSignal;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import redbacks.robot.Controllers;
import redbacks.robot.subsystems.drivetrain.DrivetrainConstants.ModulePosition;

public class DrivetrainBindings {
	public static void bind(Drivetrain drivetrain, DrivetrainHardware hardware, Controllers controllers) {
		bindControls(drivetrain, controllers);
		bindInputs(drivetrain, hardware);
		bindModules(drivetrain, hardware, controllers);

		drivetrain
		.addBinding(new ScheduledSignal(controllers.driver::getBackButton))
		.attach(hardware::reinit);
	}

	private static void bindControls(Drivetrain drivetrain, Controllers controllers) {
		drivetrain.addBinding(() -> feedDriverInputs(drivetrain, controllers.driver));

		drivetrain
		.addBinding(new ScheduledSignal(controllers.driver::getStartButton))
		.attach(() -> drivetrain.setHeading(new Rotation2d()));
	}

	private static void bindInputs(Drivetrain drivetrain, DrivetrainHardware hardware) {
		drivetrain.getYawSensor().populate(hardware.yaw);
	}

	private static void bindModules(Drivetrain drivetrain, DrivetrainHardware hardware, Controllers controllers) {
		for(ModulePosition position : ModulePosition.values()) {
			SwerveModuleBindings.bind(drivetrain.getModule(position), hardware.getModule(position));
		}
	}

	private static void feedDriverInputs(Drivetrain drivetrain, XboxController controller) {
		double forward = -controller.getRightY();
		double left = -controller.getRightX();

		// Apply linear deadzone
		double linearMagnitude = Math.sqrt(forward * forward + left * left);
		if(linearMagnitude != 0) {
			double correctedLinearMagnitude = applyJoystickDeadzone(Math.min(1, linearMagnitude));
			forward *= correctedLinearMagnitude / linearMagnitude;
			left *= correctedLinearMagnitude / linearMagnitude;
		}

		// Apply rotational deadzone
		double rotate = -applyJoystickDeadzone(controller.getLeftX());

		drivetrain.acceptDriverInputs(
			ArachneMath.signedPow(forward, JOYSTICK_EXPONENT),
			ArachneMath.signedPow(left, JOYSTICK_EXPONENT),
			ArachneMath.signedPow(rotate, JOYSTICK_EXPONENT)
		);
	}
}
