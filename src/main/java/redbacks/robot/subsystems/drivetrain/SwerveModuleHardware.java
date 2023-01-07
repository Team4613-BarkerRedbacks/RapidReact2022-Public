package redbacks.robot.subsystems.drivetrain;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;

import arachne.lib.io.GettableDouble;
import arachne.lib.io.sensors.SettableDoubleSensor;
import edu.wpi.first.wpilibj.Timer;
import redbacks.robot.Constants;
import redbacks.robot.subsystems.drivetrain.DrivetrainConstants.ModulePosition;

import static redbacks.robot.subsystems.drivetrain.SwerveModuleConstants.*;

import java.util.function.Supplier;

public class SwerveModuleHardware {
    private final ModulePosition position;
    private final CANCoder cancoder;

    final WPI_TalonFX drive, steer;
    final SettableDoubleSensor distance;
    final GettableDouble velocity, angle;

    public SwerveModuleHardware(ModulePosition position, int driveId, int steerId, int cancoderId) {
        this.position = position;
        this.cancoder = new CANCoder(cancoderId, Constants.CANIVORE_BUS_NAME);

        drive = new WPI_TalonFX(driveId, Constants.CANIVORE_BUS_NAME);
        steer = new WPI_TalonFX(steerId, Constants.CANIVORE_BUS_NAME);

        distance = SettableDoubleSensor.create(drive::getSelectedSensorPosition, drive::setSelectedSensorPosition);
        velocity = () -> drive.getSelectedSensorVelocity() * ENCODER_VELOCITY_TO_METRES_PER_SECOND_MULTIPLIER;
        angle = () -> steer.getSelectedSensorPosition() * STEER_TICKS_TO_DEGREES;
    }

    public void initialize() {
        // Drive motor
        drive.configFactoryDefault(Constants.CAN_TIMEOUT);

        drive.setNeutralMode(NeutralMode.Brake);
        drive.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        drive.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 5, Constants.CAN_TIMEOUT);

        drive.selectProfileSlot(0, 0);
        drive.config_kP(0, VELOCITY_KP, Constants.CAN_TIMEOUT);
        drive.config_kI(0, VELOCITY_KI, Constants.CAN_TIMEOUT);
        drive.config_kD(0, VELOCITY_KD, Constants.CAN_TIMEOUT);
        drive.config_kF(0, VELOCITY_KF, Constants.CAN_TIMEOUT);
        drive.configClosedLoopPeakOutput(0, VELOCITY_MAX_PID_OUTPUT, Constants.CAN_TIMEOUT);

        // Steer motor
        steer.configFactoryDefault(Constants.CAN_TIMEOUT);

        steer.setNeutralMode(NeutralMode.Brake);
        steer.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        steer.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 5, Constants.CAN_TIMEOUT);

        steer.selectProfileSlot(0, 0);
        steer.config_kP(0, STEER_KP, Constants.CAN_TIMEOUT);
        steer.config_kI(0, STEER_KI, Constants.CAN_TIMEOUT);
        steer.config_kD(0, STEER_KD, Constants.CAN_TIMEOUT);

        // CANCoder
        retryOnError(() -> cancoder.configFactoryDefault(Constants.CANCODER_TIMEOUT));
        retryOnError(() -> cancoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180, Constants.CANCODER_TIMEOUT));
        retryOnError(() -> cancoder.configMagnetOffset(ANGLE_OFFSET_FOR(position), Constants.CANCODER_TIMEOUT));
        retryOnError(() -> cancoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, Constants.CANCODER_STATUS_FRAME_PERIOD));
        retryOnError(() -> cancoder.setStatusFramePeriod(CANCoderStatusFrame.VbatAndFaults, Constants.CANCODER_STATUS_FRAME_PERIOD));

        retryOnError(() -> steer.setSelectedSensorPosition(cancoder.getAbsolutePosition() / STEER_TICKS_TO_DEGREES));
    }

    public void resetModuleAngle() {
        steer.setSelectedSensorPosition(cancoder.getAbsolutePosition() / STEER_TICKS_TO_DEGREES);
    }

    private void retryOnError(Supplier<ErrorCode> configure) {
        ErrorCode result = configure.get();

        while(result != ErrorCode.OK) {
            System.err.println("Configuration of " + position + " module failed with result '" + result + "', retrying in 100ms...");
            Timer.delay(0.1);
            result = configure.get();
        }
    }
}
