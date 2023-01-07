package redbacks.robot.subsystems.drivetrain;

import com.kauailabs.navx.frc.AHRS;

import arachne.lib.io.Gettable;
import arachne.lib.io.GettableDouble;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.SPI;

import static redbacks.robot.subsystems.drivetrain.DrivetrainConstants.*;

import java.security.InvalidParameterException;

public class DrivetrainHardware {
    // ----------------------------------------
    // Modules
    // ----------------------------------------

    private SwerveModuleHardware
        moduleFL = new SwerveModuleHardware(ModulePosition.FRONT_LEFT, 0, 10, 0),
        moduleBL = new SwerveModuleHardware(ModulePosition.BACK_LEFT, 1, 11, 1),
        moduleBR = new SwerveModuleHardware(ModulePosition.BACK_RIGHT, 2, 12, 2),
        moduleFR = new SwerveModuleHardware(ModulePosition.FRONT_RIGHT, 3, 13, 3);

    SwerveModuleHardware getModule(ModulePosition position) {
        switch(position) {
            case BACK_LEFT: return moduleBL;
            case BACK_RIGHT: return moduleBR;
            case FRONT_LEFT: return moduleFL;
            case FRONT_RIGHT: return moduleFR;
            default: throw new InvalidParameterException("Invalid module position");
        }
    }

    // ----------------------------------------
    // Sensors
    // ----------------------------------------

    public final AHRS navx = new AHRS(SPI.Port.kMXP);
    public final Gettable<Rotation2d> yaw = () -> Rotation2d.fromDegrees(-navx.getYaw());
    public final GettableDouble pitchDegrees = navx::getPitch;

    // -------------------------------------------------------------------------

    public void initialize() {
        for(ModulePosition position : ModulePosition.values()) getModule(position).initialize();
    }

    public void reinit() {
        for(ModulePosition position : ModulePosition.values()) getModule(position).resetModuleAngle();
    }
}
