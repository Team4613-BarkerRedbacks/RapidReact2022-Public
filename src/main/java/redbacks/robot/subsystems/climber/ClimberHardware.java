package redbacks.robot.subsystems.climber;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import arachne.lib.io.GettableBoolean;
import arachne.lib.io.sensors.SettableDoubleSensor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import redbacks.robot.Constants;

import static redbacks.robot.subsystems.climber.ClimberConstants.*;

public class ClimberHardware {
    public final WPI_TalonFX
        mainHooks = new WPI_TalonFX(4),
        mainHooksFollower = new WPI_TalonFX(5);

    public final Solenoid
        secondaryHooksSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 7),
        unlockSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 5);

    private final DigitalInput
        leftBottomSensorReleased = new DigitalInput(0),
        rightBottomSensorReleased = new DigitalInput(1);

    public final GettableBoolean atBottomSensor = () -> !leftBottomSensorReleased.get() || !rightBottomSensorReleased.get();

    public final SettableDoubleSensor positionSensor = SettableDoubleSensor.create(
        () -> mainHooks.getSelectedSensorPosition() * TICKS_TO_METRES,
        (metres) -> mainHooks.setSelectedSensorPosition(metres / TICKS_TO_METRES, 0, Constants.CAN_TIMEOUT)
    );

    public void initialize() {
        mainHooks.configFactoryDefault(Constants.CAN_TIMEOUT);
        mainHooks.setNeutralMode(NeutralMode.Brake);
        mainHooks.setInverted(true);
        mainHooks.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, Constants.CAN_TIMEOUT);

        mainHooks.config_kP(0, KP, Constants.CAN_TIMEOUT);
        mainHooks.config_kI(0, KI, Constants.CAN_TIMEOUT);
        mainHooks.config_kD(0, KD, Constants.CAN_TIMEOUT);
        mainHooks.configClosedLoopPeakOutput(0, MAX_PID_OUTPUT, Constants.CAN_TIMEOUT);

        mainHooksFollower.configFactoryDefault(Constants.CAN_TIMEOUT);
        mainHooksFollower.setNeutralMode(NeutralMode.Brake);
        mainHooksFollower.setInverted(true);
        mainHooksFollower.follow(mainHooks);

        positionSensor.accept(HOOKS_AT_BOTTOM_HEIGHT_METRES);
    }
}
 