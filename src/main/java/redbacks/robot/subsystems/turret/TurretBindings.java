package redbacks.robot.subsystems.turret;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;

import static redbacks.robot.subsystems.turret.TurretConstants.*;

public class TurretBindings {
    public static void bind(Turret turret, TurretHardware hardware) {
        bindInputs(turret, hardware);
        bindOutputs(turret, hardware);
    }

    private static void bindOutputs(Turret turret, TurretHardware hardware) {
        turret.getTargetYawAndFeedForwardOutput().attach((positionAndFeedForward) -> hardware.turret.set(
            // TalonFXControlMode.PercentOutput,
            TalonFXControlMode.MotionMagic,
            positionAndFeedForward.position / TURRET_TICKS_TO_DEGREES,
            DemandType.ArbitraryFeedForward,
            positionAndFeedForward.feedForward
        ));

        turret.getTargetPitch().attach((pitchDegrees) -> hardware.hood.set(
            ControlMode.MotionMagic,
            (pitchDegrees - HOOD_MAX_ANGLE_DEGREES + HOOD_ABSOLUTE_READING_AT_BASE) / HOOD_TICKS_TO_DEGREES
        ));
    }

    private static void bindInputs(Turret turret, TurretHardware hardware) {
        turret.getYawSensor().populate(hardware.yaw);
        turret.getPitchSensor().populate(hardware.pitch);
        turret.getYawVelocityDegreesPerSecSensor().populate(hardware.yawVelocity);
    }
}