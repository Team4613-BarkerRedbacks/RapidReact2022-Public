package redbacks.robot.subsystems.turret;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import arachne.lib.io.GettableDouble;
import arachne.lib.io.sensors.SettableDoubleSensor;
import redbacks.robot.Constants;

import static redbacks.robot.subsystems.turret.TurretConstants.*;

public class TurretHardware {
    public final WPI_TalonFX turret = new WPI_TalonFX(14);
    public final WPI_TalonSRX hood = new WPI_TalonSRX(15);

    public final SettableDoubleSensor yaw = SettableDoubleSensor.create(
        () -> turret.getSelectedSensorPosition() * TURRET_TICKS_TO_DEGREES,
        (degrees) -> turret.setSelectedSensorPosition(degrees / TURRET_TICKS_TO_DEGREES, 0, Constants.CAN_TIMEOUT)
    );

    public final GettableDouble yawVelocity = () -> turret.getSelectedSensorVelocity() * 10 * TURRET_TICKS_TO_DEGREES;
    public final GettableDouble pitch = () -> hood.getSelectedSensorPosition() * HOOD_TICKS_TO_DEGREES - HOOD_ABSOLUTE_READING_AT_BASE + HOOD_MAX_ANGLE_DEGREES;

    public void initialize() {
        turret.configFactoryDefault(Constants.CAN_TIMEOUT);

        turret.setNeutralMode(NeutralMode.Brake);
        turret.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        turret.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 5, Constants.CAN_TIMEOUT);

        turret.setInverted(true);
        turret.selectProfileSlot(0, 0);
        turret.config_kP(0, TURRET_KP, Constants.CAN_TIMEOUT);
        turret.config_kI(0, TURRET_KI, Constants.CAN_TIMEOUT);
        turret.config_kD(0, TURRET_KD, Constants.CAN_TIMEOUT);
        turret.config_kF(0, TURRET_KF, Constants.CAN_TIMEOUT);
        turret.configMotionAcceleration(TURRET_MAX_ACCELERATION_DEGREES_PER_SEC_SQUARED / TURRET_TICKS_TO_DEGREES / 10, Constants.CAN_TIMEOUT);
        turret.configMotionCruiseVelocity(TURRET_CRUISE_VELOCITY_DEGREES_PER_SEC / TURRET_TICKS_TO_DEGREES / 10);
        // turret.configClosedLoopPeakOutput(0, TURRET_MAX_PID_OUTPUT, Constants.CAN_TIMEOUT);

        hood.setNeutralMode(NeutralMode.Brake);
        hood.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);

        hood.selectProfileSlot(0, 0);
        hood.config_kP(0, HOOD_KP, Constants.CAN_TIMEOUT);
        hood.config_kI(0, HOOD_KI, Constants.CAN_TIMEOUT);
        hood.config_kD(0, HOOD_KD, Constants.CAN_TIMEOUT);
        hood.config_kF(0, HOOD_KF, Constants.CAN_TIMEOUT);
        hood.configMotionAcceleration(HOOD_MAX_ACCELERATION_DEGREES_PER_SEC_SQUARED / HOOD_TICKS_TO_DEGREES / 10, Constants.CAN_TIMEOUT);
        hood.configMotionCruiseVelocity(HOOD_CRUISE_VELOCITY_DEGREES_PER_SEC / HOOD_TICKS_TO_DEGREES / 10);

        yaw.accept(0);
    }
}
