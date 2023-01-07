package redbacks.robot.subsystems.aiming;

import arachne.lib.dashboard.Dashboard;
import arachne.lib.immutables.FlaggedDoubleValue;
import arachne.lib.immutables.FlaggedValue;
import arachne.lib.immutables.Pair;
import arachne.lib.io.Gettable;
import arachne.lib.io.GettableBoolean;
import arachne.lib.io.GettableDouble;
import arachne.lib.io.sensors.GettableBooleanInput;
import arachne.lib.io.sensors.GettableDoubleInput;
import arachne.lib.io.sensors.GettableInput;
import arachne.lib.io.sensors.Input;
import arachne.lib.pipeline.DoublePipe;
import arachne.lib.pipeline.DoubleSource;
import arachne.lib.pipeline.Pipe;
import arachne.lib.pipeline.Source;
import arachne.lib.states.State;
import arachne.lib.states.StatefulSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import redbacks.robot.subsystems.aiming.AimingRings.Target;
import redbacks.robot.subsystems.turret.TurretConstants;
import redbacks.robot.subsystems.turret.Turret.TurretControlInput;

import static redbacks.robot.subsystems.aiming.AimingConstants.*;

public class Aiming extends StatefulSubsystem<Aiming.SystemState, Aiming> {
    private final Pipe<VisionCalculatedPosition> position = new Pipe<VisionCalculatedPosition>(new VisionCalculatedPosition(new Pose2d(), 0));
    private final Pipe<TurretControlInput> targetTurretControlOutput = new Pipe<TurretControlInput>(new TurretControlInput(0, 0));
    private final DoublePipe targetHoodAngle = new DoublePipe(0);
    private final Pipe<FlaggedDoubleValue> shootingSpeedOutput = new Pipe<FlaggedDoubleValue>(new FlaggedDoubleValue(0, false));

    private final GettableInput<Pose2d> drivetrainPosition = new GettableInput<Pose2d>();
    private final GettableDoubleInput hubRelativeYawDegrees = new GettableDoubleInput();
    private final GettableDoubleInput turretYaw = new GettableDoubleInput();
    private final GettableDoubleInput hoodPitch = new GettableDoubleInput();
    private final GettableInput<Pair<Double, ChassisSpeeds>> drivetrainState = new GettableInput<Pair<Double, ChassisSpeeds>>();
    private final GettableBooleanInput ballHandlingHasRightColour = new GettableBooleanInput();
    private final GettableBooleanInput isStrategyShootWhileMoving = new GettableBooleanInput();

    public Aiming() {
        super(SystemState.RUNNING, SystemState.class);
    }

    @Override
    protected Aiming getSelf() {
        return this;
    }

    enum SystemState implements State<SystemState, Aiming> {
        RUNNING {
            @Override
            void recalculatePosition(Aiming aiming, double horizontalAngleRadians, double verticalAngleRadians, double latencySeconds) {
                Rotation2d drivetrainHeading = aiming.drivetrainPosition.get().getRotation();
                Rotation2d turretHeading = Rotation2d.fromDegrees(aiming.turretYaw.get());

                double distanceMetres = HUB_RADIUS_METRES + LIMELIGHT_TO_HUB_HEIGHT_METRES / Math.tan(verticalAngleRadians) / Math.cos(horizontalAngleRadians);
                Rotation2d fieldRelativeAngle = drivetrainHeading.plus(new Rotation2d(horizontalAngleRadians)).plus(turretHeading);

                Dashboard.getInstance().putNumber("Distance to center", distanceMetres);
                Dashboard.getInstance().putNumber("Angle to center", fieldRelativeAngle.getDegrees());

                Translation2d limelightOffsetFromHub = new Translation2d(
                    -distanceMetres,
                    fieldRelativeAngle
                );

                Translation2d turretOffsetFromLimelight = new Translation2d(
                    -TURRET_OFFSET_FROM_LIMELIGHT_METRES,
                    drivetrainHeading.plus(turretHeading)
                );

                Translation2d robotOffsetFromTurret = new Translation2d(
                    ROBOT_CENTRE_OFFSET_FROM_TURRET_METRES,
                    drivetrainHeading
                );

                Translation2d location = limelightOffsetFromHub
                    .plus(turretOffsetFromLimelight)
                    .plus(robotOffsetFromTurret);

                aiming.position.accept(new VisionCalculatedPosition(
                    new Pose2d(location.getX(), location.getY(), drivetrainHeading),
                    latencySeconds
                ));
            }

            @Override
            public void run(Aiming aiming) {
                TURRET_ONLY.run(aiming);
            }
        },
        TURRET_ONLY {
            @Override
            public void run(Aiming aiming) {
                var drivetrainState = aiming.drivetrainState.get();

                FlaggedValue<Target> target = AimingRings.getTarget(
                    drivetrainState.getSecond().vxMetersPerSecond,
                    drivetrainState.getSecond().vyMetersPerSecond,
                    drivetrainState.getFirst(),
                    !aiming.isStrategyShootWhileMoving.get(),
                    aiming.isStrategyShootWhileMoving.get()
                );

                Dashboard.getInstance().putBoolean("In shooting range", target.getFlag());

                Translation2d position = aiming.drivetrainPosition.get().getTranslation();
    
                double robotToHubDirection = Math.atan2(-position.getY(), -position.getX());
                double hubRelativeYawDegrees = aiming.hubRelativeYawDegrees.get();
                double turretTargetAngleDegrees = target.getValue().horizontalOffsetDegrees - hubRelativeYawDegrees;
                double turretAngleToleranceDegrees;

                double semiHubAngleRange = Math.toDegrees(Math.atan2(HUB_RADIUS_METRES, drivetrainState.getFirst() /* distance to hub */));

                if(aiming.ballHandlingHasRightColour.get()) {
                    turretAngleToleranceDegrees = semiHubAngleRange * AIMING_TOLERANCE_MULTIPLIER;
                }
                else {
                    double discardOffset = robotToHubDirection > 0 ? semiHubAngleRange + DISCARD_TURRET_TOLERANCE_DEGREES : -semiHubAngleRange - DISCARD_TURRET_TOLERANCE_DEGREES;
                    if(!TurretConstants.isAngleInRange(turretTargetAngleDegrees)) discardOffset = -discardOffset;

                    turretTargetAngleDegrees += discardOffset;
                    turretAngleToleranceDegrees = DISCARD_TURRET_TOLERANCE_DEGREES;
                }

                double turretAngleError = Math.abs(turretTargetAngleDegrees - aiming.turretYaw.get()) % 360;
                if(turretAngleError > 180) turretAngleError = 360 - turretAngleError;

                double robotAngleToHubRateOfChange = Math.toDegrees(
                    (position.getY() * drivetrainState.getSecond().vxMetersPerSecond - position.getX() * drivetrainState.getSecond().vyMetersPerSecond)
                    / (Math.pow(position.getX(), 2) + Math.pow(position.getY(), 2))
                    - drivetrainState.getSecond().omegaRadiansPerSecond
                );
    
                Dashboard.getInstance().putNumber("Turret velocity compensation", robotAngleToHubRateOfChange);
                Dashboard.getInstance().putNumber("Turret tolerance", turretAngleToleranceDegrees);

                aiming.targetTurretControlOutput.accept(new TurretControlInput(
                    turretTargetAngleDegrees,
                    robotAngleToHubRateOfChange
                ));
    
                double hoodTargetAngleDegrees = target.getValue().verticalDegrees;
    
                aiming.targetHoodAngle.accept(hoodTargetAngleDegrees);

                boolean isHoodOnTarget = Math.abs(hoodTargetAngleDegrees - aiming.hoodPitch.get()) < HOOD_ANGLE_TOLERANCE_DEGREES
                    || !aiming.ballHandlingHasRightColour.get();

                aiming.shootingSpeedOutput.accept(new FlaggedDoubleValue(
                    target.getValue().shooterSpeedMps,
                    (target.getFlag() || !aiming.ballHandlingHasRightColour.get())
                        && turretAngleError < turretAngleToleranceDegrees
                        && isHoodOnTarget
                ));
            }
        },
        PAUSED {
            @Override
            void recalculatePosition(Aiming aiming, double horizontalAngleRadians, double verticalAngleRadians, double latencySeconds) {}
        },
        STOW_FOR_CLIMB {
            @Override
            public void run(Aiming aiming) {
                aiming.shootingSpeedOutput.accept(new FlaggedDoubleValue(0, false));
                aiming.targetTurretControlOutput.accept(new TurretControlInput(TURRET_STOW_POSITION_DEGREES, 0));
            }

            @Override
            void recalculatePosition(Aiming aiming, double horizontalAngleRadians, double verticalAngleRadians, double latencySeconds) {}
        },
        OVERRIDE {
            @Override
            public void run(Aiming aiming) {
                aiming.shootingSpeedOutput.accept(new FlaggedDoubleValue(6, true));
                aiming.targetTurretControlOutput.accept(new TurretControlInput(0, 0));
            }

            @Override
            void recalculatePosition(Aiming aiming, double horizontalAngleRadians, double verticalAngleRadians, double latencySeconds) {}
        };

        void recalculatePosition(Aiming aiming, double horizontalAngleRadians, double verticalAngleRadians, double latencySeconds) {}
    }

    public void recalculatePosition(double horizontalAngleRadians, double verticalAngleRadians, double latencySeconds) {
        state.recalculatePosition(this, horizontalAngleRadians, verticalAngleRadians, latencySeconds);
    }

    public void pause() {
        setState(SystemState.PAUSED);
    }

    public void pauseOdometry() {
        if(getState().equals(SystemState.RUNNING)) setState(SystemState.TURRET_ONLY);
    }

    public void resume() {
        setState(SystemState.RUNNING);
    }

    public void stowForClimb() {
        setState(SystemState.STOW_FOR_CLIMB);
    }

    public Source<VisionCalculatedPosition> getPositionOutput() {
        return position;
    }

    public Input<Gettable<Pose2d>> getDrivetrainPositionSensor() {
        return drivetrainPosition;
    }

    public GettableDoubleInput getTurretYawSensor() {
        return turretYaw;
    }

    public Input<GettableDouble> getHoodPitchSensor() {
        return hoodPitch;
    }

    public GettableDoubleInput getHubRelativeYawDegrees() {
        return hubRelativeYawDegrees;
    }

    public Source<TurretControlInput> getTurretControlOutput() {
        return targetTurretControlOutput;
    }

    public DoubleSource getHoodAngleOutput() {
        return targetHoodAngle;
    }

    public GettableInput<Pair<Double, ChassisSpeeds>> getDrivetrainState() {
        return drivetrainState;
    }

    public Pipe<FlaggedDoubleValue> getShootingSpeedOutput() {
        return shootingSpeedOutput;
    }

    public Input<GettableBoolean> getBallHandlingHasRightColourSensor() {
        return ballHandlingHasRightColour;
    }

    public Input<GettableBoolean> getIsStrategyShootWhileMovingSensor() {
        return isStrategyShootWhileMoving;
    }
}
