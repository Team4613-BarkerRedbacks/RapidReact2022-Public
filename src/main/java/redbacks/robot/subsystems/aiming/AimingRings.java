package redbacks.robot.subsystems.aiming;

import java.io.BufferedReader;
import java.io.File;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.FileInputStream;
import java.util.HashMap;
import java.util.Map;

import arachne.lib.dashboard.Dashboard;
import arachne.lib.immutables.FlaggedValue;
import edu.wpi.first.wpilibj.DriverStation;

import static redbacks.robot.subsystems.aiming.AimingConstants.*;

public class AimingRings {
    static HashMap<Integer, Map<Integer, Ring[]>> shootWhileMovingMap;

    static Ring[] shootWhileStationaryRings = {
        new Ring(2.33f, 5f-0.2f, 0, 64),
        new Ring(2.87f, 5.4f+0.15f-0.2f, 0, 64),
        new Ring(3.83f, 6f, 0, 62.5f),
        new Ring(4.58f, 6.3f+0.8f-0.6f, 0, 57),
        new Ring(5.48f, 7f+1-0.7f, 0, 56),
        new Ring(5.72f, 7.2f+1-0.7f, 0, 55),
    };

    static Ring[] autoRings = {
        // new Ring(2.33f, 5f-0.7f, 0, 64),
        // new Ring(2.87f, 5.4f+0.15f-0.7f, 0, 64),
        new Ring(3.0f, 5.5f, 0, 64),
        new Ring(3.4f, 5.5f, 0, 64),
        new Ring(3.83f, 6f-0.4f, 0, 61),
        new Ring(4.58f, 6.3f+0.8f-0.6f, 0, 56),
        new Ring(5.48f, 7f+1-0.7f, 0, 54),
        new Ring(5.72f, 7.2f+1-0.8f, 0, 54),
    };

    public static void initialize(File mapFile) {
        try(BufferedReader br = new BufferedReader(new InputStreamReader(new FileInputStream(mapFile), "UTF8"))) {
            shootWhileMovingMap = getAimingMapFromFile(br);
        }
        catch(IOException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
    }

    private static HashMap<Integer, Map<Integer, Ring[]>> getAimingMapFromFile(BufferedReader br) throws IOException {
        var map = new HashMap<Integer, Map<Integer, Ring[]>>();
        String line;
        while((line = br.readLine()) != null) {
            String values[] = line.split(";");
            int velocityX = (Integer.parseInt(values[0]));

            var innerMap = new HashMap<Integer, Ring[]>();
            for(int i = 1; i < values.length; i++) {
                String[] subValues = values[i].split(",");
                Ring[] rings = new Ring[subValues.length / 4];
                
                for(int j = 1; j < subValues.length; j += 4) {
                    rings[j / 4] = // this quotient will not mathematically yeild a whole number, but after Java cuts off everything past the decimal point (flooring it) it will be the correct index. This logic also applies to the "new Ring[subValues.length / 3]" above.
                        new Ring(Float.parseFloat(subValues[j]), Float.parseFloat(subValues[j + 1]), Float.parseFloat(subValues[j + 2]), Float.parseFloat(subValues[j + 3]));
                }

                int velocityY = Integer.parseInt(subValues[0]);
                innerMap.put(velocityY, rings);
            }
            map.put(velocityX, innerMap);
        }
        return map;
    }

    static FlaggedValue<Target> getTarget(double velocityX, double velocityY, double distanceToHub, boolean forceStationary, boolean forceShootWhileMoving) {
        int velocityXmmps = Math.round(Math.round(velocityX * 1000 / VELOCITY_X_MAP_INCREMENT_MM) * VELOCITY_X_MAP_INCREMENT_MM);
        int velocityYmmps = Math.round(Math.round(velocityY * 1000 / VELOCITY_Y_MAP_INCREMENT_MM) * VELOCITY_Y_MAP_INCREMENT_MM);

        Ring[] rings;
        boolean closeShooting = false;
        boolean farShooting = false;
        boolean shootWhileMoving;

        if(forceShootWhileMoving) shootWhileMoving = true;
        else if(forceStationary) shootWhileMoving = false;
        else {
            closeShooting = distanceToHub < MIN_SHOOT_WHILE_MOVING_DISTANCE_METRES;
            farShooting = distanceToHub > MAX_SHOOT_WHILE_MOVING_DISTANCE_METRES;
            shootWhileMoving = !closeShooting && !farShooting;
        }

        if(shootWhileMoving) rings = shootWhileMovingMap.get(velocityXmmps).get(Math.abs(velocityYmmps));
        else if(!DriverStation.isAutonomous()) rings = shootWhileStationaryRings;
        else rings = autoRings;

        // Finds closest smaller and larger ring
        Ring closestSmallerRing = null;
        Ring closestLargerRing = null;

        for(int i = 0; i < rings.length && closestLargerRing == null; i++) {
            if (rings[i].distance < distanceToHub) closestSmallerRing = rings[i];
            else closestLargerRing = rings[i];
        }

        Dashboard.getInstance().putNumber("swm vx", velocityXmmps);
        Dashboard.getInstance().putNumber("swm vy", velocityYmmps);

        Dashboard.getInstance().putNumber("swm vxmps", velocityX);
        Dashboard.getInstance().putNumber("swm vymps", velocityY);

        try {
            // Checks if we dont have a closest or further shooting ring and returns the closest ring we are closer too
            if(closestSmallerRing == null) return new FlaggedValue<Target>(closestLargerRing.target, false);
            if(closestLargerRing == null) return new FlaggedValue<Target>(closestSmallerRing.target, false);
        } 
        catch (NullPointerException e) {
            System.err.println("rvx: " + velocityXmmps + " rvy: " + velocityYmmps + " radius: " + distanceToHub);
            for(Ring ring : rings) System.err.println(ring);
            throw e;
        }

        // Uses a linear equation to estimate the right speed
        double shootingSpeedGradient = (closestLargerRing.target.shooterSpeedMps - closestSmallerRing.target.shooterSpeedMps) / (closestLargerRing.distance - closestSmallerRing.distance);
        double shootingSpeedIntercept = closestSmallerRing.target.shooterSpeedMps - closestSmallerRing.distance * shootingSpeedGradient;

        float targetShootingSpeed = (float) (distanceToHub * shootingSpeedGradient + shootingSpeedIntercept);

        double horizontalOffsetGradient = (closestLargerRing.target.horizontalOffsetDegrees - closestSmallerRing.target.horizontalOffsetDegrees) / (closestLargerRing.distance - closestSmallerRing.distance);
        double horizontalOffsetIntercept = closestSmallerRing.target.horizontalOffsetDegrees - closestSmallerRing.distance * horizontalOffsetGradient;

        float targetHorizontalOffsetDegrees = (float) (distanceToHub * horizontalOffsetGradient + horizontalOffsetIntercept);

        double verticalAngleGradient = (closestLargerRing.target.verticalDegrees - closestSmallerRing.target.verticalDegrees) / (closestLargerRing.distance - closestSmallerRing.distance);
        double verticalAngleIntercept = closestSmallerRing.target.verticalDegrees - closestSmallerRing.distance * verticalAngleGradient;

        float targetVerticalAngleDegrees = (float) (distanceToHub * verticalAngleGradient + verticalAngleIntercept);

        boolean flag = true;
        double robotVelocitySquared = velocityX * velocityX + velocityY * velocityY;
        // Logic: sqrt(x^2 + y^2) < c is the same as x^2 + y^2 < c^2
        if(forceStationary || farShooting) {
            if(robotVelocitySquared > FAR_MAX_ROBOT_SPEED_STATIONARY_SHOOTING * FAR_MAX_ROBOT_SPEED_STATIONARY_SHOOTING) {
                flag = false;
            }
        }
        else if(closeShooting) {
            if(robotVelocitySquared > CLOSE_MAX_ROBOT_SPEED_STATIONARY_SHOOTING * CLOSE_MAX_ROBOT_SPEED_STATIONARY_SHOOTING) {
                flag = false;
            }
        }

        return new FlaggedValue<Target>(new Target(
            targetShootingSpeed,
            -Math.copySign(targetHorizontalOffsetDegrees, velocityYmmps),
            targetVerticalAngleDegrees
        ), flag);
    }

    public static double linearlyInterpolate(double value, double value1, double value2, double output1, double output2) { // TODO this should probably be moved to a different file i.e. ArachneMath
        double gradient = (output2 - value2) / (output1 - value1);
        double intercept = output1 - value1 * gradient;

        double output = value * gradient + intercept;
        return output;
    }

    protected static class Ring {
        final float distance;
        final Target target;

        Ring(float distance, float shooterSpeedMps, float turretOffsetDegrees, float hoodDegrees) {
            this.distance = distance;
            target = new Target(shooterSpeedMps, turretOffsetDegrees, hoodDegrees);
        }

        @Override
        public String toString() {
            return "Ring(radius: " + distance + ", shooter speed: " + target.shooterSpeedMps + ", offset degrees: " + target.horizontalOffsetDegrees + ")";
        }
    }

    protected static class Target {
        final float shooterSpeedMps, horizontalOffsetDegrees, verticalDegrees;

        Target(float shooterSpeedMps, float horizontalOffsetDegrees, float verticalDegrees) {
            this.shooterSpeedMps = shooterSpeedMps;
            this.horizontalOffsetDegrees = horizontalOffsetDegrees;
            this.verticalDegrees = verticalDegrees;
        }
    }
}