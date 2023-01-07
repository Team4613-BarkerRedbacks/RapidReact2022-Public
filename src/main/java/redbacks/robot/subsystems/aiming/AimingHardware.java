package redbacks.robot.subsystems.aiming;

import org.photonvision.PhotonCamera;
import org.photonvision.common.hardware.VisionLEDMode;

public class AimingHardware {
    PhotonCamera limelight = new PhotonCamera("limelight");

    public void initialize() {
        limelight.setDriverMode(false);
        limelight.setLED(VisionLEDMode.kOn);
    }
}
