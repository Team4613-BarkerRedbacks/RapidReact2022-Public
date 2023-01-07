package redbacks.robot.subsystems.ballhandling;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import arachne.lib.io.Gettable;
import arachne.lib.io.GettableBoolean;
import arachne.lib.io.sensors.ResettableDoubleSensor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import redbacks.robot.Constants;

public class BallHandlingHardware {
    public final WPI_TalonFX
        intake = new WPI_TalonFX(6),
        tower = new WPI_TalonFX(7);

    public final Solenoid intakeExtension = new Solenoid(PneumaticsModuleType.CTREPCM, 6);

    private final DigitalInput
        intakeSensor = new DigitalInput(2),
        towerSensor = new DigitalInput(3),
        bottomSensor = new DigitalInput(4);

    public final GettableBoolean
        ballAtTop = towerSensor::get,
        ballAtBottom = () -> bottomSensor.get() || intakeSensor.get();

    public final ResettableDoubleSensor
        towerPositionTicks = ResettableDoubleSensor.create(
            tower::getSelectedSensorPosition,
            () -> tower.setSelectedSensorPosition(0)
        );

    private final DigitalInput
        redSensor = new DigitalInput(5),
        blueSensor = new DigitalInput(6);

    public final Gettable<BallColour> currentColour = () ->
            blueSensor.get() ? BallColour.BLUE
        : redSensor.get() ? BallColour.RED
        : BallColour.NONE;

    public void initialize() {
        intake.configFactoryDefault(Constants.CAN_TIMEOUT);
		intake.setNeutralMode(NeutralMode.Coast);

        tower.configFactoryDefault(Constants.CAN_TIMEOUT);
		tower.setNeutralMode(NeutralMode.Brake);
        tower.setInverted(true);

        tower.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, Constants.CAN_TIMEOUT);
        tower.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 5, Constants.CAN_TIMEOUT);
        towerPositionTicks.reset();
    }
}
