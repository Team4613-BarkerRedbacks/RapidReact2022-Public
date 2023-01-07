package redbacks.robot.subsystems.aiming;

import java.util.HashMap;

import org.junit.jupiter.api.Test;

public class TestShootWhileMovingMap {
    @Test
    void testMapAccess() {
        var map = new HashMap<Float, HashMap<Float, ShootingParams[]>>();

        map.put(0.1f, new HashMap<Float, ShootingParams[]>());
        map.get(0.1f).put(0.2f, new ShootingParams[] {
            new ShootingParams(5f, 7f, 3.5f),
            new ShootingParams(6f, 8f, 2.5f),
            new ShootingParams(7f, 9f, 1f),
            new ShootingParams(8f, 10f, 0f)
        });

        ShootingParams[] rings = map.get(0.1f).get(0.2f);
        int closestIndex = 0;
        float robotDist = 6.7f;
        for (int i = 0; i < rings.length; i++) {
            if (Math.abs(rings[i].distance - robotDist) < Math.abs(rings[closestIndex].distance - robotDist)) {
                closestIndex = i;
            };
        }
        
        System.out.println(rings[closestIndex].distance);
        System.out.println(rings[closestIndex].shooterSpeedMps);
        System.out.println(rings[closestIndex].turretOffsetDegrees);
    }

    public static class ShootingParams {
        float distance, shooterSpeedMps, turretOffsetDegrees;

        ShootingParams(float distance, float shooterSpeedMps, float turretOffsetDegrees) {
            this.distance = distance;
            this.shooterSpeedMps = shooterSpeedMps;
            this.turretOffsetDegrees = turretOffsetDegrees;
        }
    }
}
