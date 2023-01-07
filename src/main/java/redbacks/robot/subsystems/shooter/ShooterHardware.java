package redbacks.robot.subsystems.shooter;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.RobotController;
import redbacks.robot.Constants;

import static redbacks.robot.subsystems.shooter.ShooterConstants.*;

public class ShooterHardware {
    public final WPI_TalonFX
	    shooter = new WPI_TalonFX(8),
        shooterFollower = new WPI_TalonFX(9);

    public void initialize() {
        shooter.configFactoryDefault(Constants.CAN_TIMEOUT);
		shooter.setNeutralMode(NeutralMode.Coast);

		shooter.configOpenloopRamp(RAMP_TIME_SECONDS);
		shooter.configClosedloopRamp(RAMP_TIME_SECONDS);

		shooter.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        shooter.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 5, Constants.CAN_TIMEOUT);

        shooter.selectProfileSlot(0, 0);
        shooter.config_kP(0, KP, Constants.CAN_TIMEOUT);
        shooter.config_kI(0, KI, Constants.CAN_TIMEOUT);
        shooter.config_kD(0, KD, Constants.CAN_TIMEOUT);
        shooter.configClosedLoopPeakOutput(0, MAX_PID_OUTPUT, Constants.CAN_TIMEOUT);

        shooterFollower.configFactoryDefault(Constants.CAN_TIMEOUT);
		shooterFollower.setNeutralMode(NeutralMode.Coast);
        shooterFollower.setInverted(true);
        shooterFollower.follow(shooter);
    }

    public double getFeedForwardCoefficient() {
        // double currentVoltage = powerDistributionHub.getVoltage();
        double currentVoltage = RobotController.getBatteryVoltage();
        double shooterSpeed = 1594.6 * currentVoltage - 2001;
        double feedForward = 0.8 / shooterSpeed;

        return feedForward;
    }
}