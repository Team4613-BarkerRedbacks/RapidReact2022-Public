package redbacks.robot.subsystems.aiming;

import java.io.File;

import org.junit.jupiter.api.Test;

import edu.wpi.first.wpilibj.Filesystem;

public class TestAimingRings {
    @Test
    void timeInitialize() throws Exception {
        AimingRings.initialize(new File(Filesystem.getDeployDirectory(), "AimingMap.csv"));
    }
}