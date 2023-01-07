package redbacks.robot.subsystems.shooter;

import static redbacks.robot.subsystems.shooter.ShooterConstants.*;

import arachne.lib.dashboard.Dashboard;
import arachne.lib.immutables.FlaggedDoubleValue;
import arachne.lib.io.GettableBoolean;
import arachne.lib.io.GettableDouble;
import arachne.lib.io.sensors.GettableBooleanInput;
import arachne.lib.io.sensors.GettableDoubleInput;
import arachne.lib.io.sensors.Input;
import arachne.lib.pipeline.BooleanPipe;
import arachne.lib.pipeline.BooleanSource;
import arachne.lib.pipeline.DoublePipe;
import arachne.lib.pipeline.DoubleSource;
import arachne.lib.pipeline.filters.ChangedBooleanFilter;
import arachne.lib.states.State;
import arachne.lib.states.StatefulSubsystem;

public class Shooter extends StatefulSubsystem<Shooter.SystemState, Shooter> {
    private final DoublePipe targetVelocityMps;
    private final BooleanPipe isReadyToBeFed;

    private final GettableDoubleInput velocityMps, distanceToHubMetres;
    private final GettableBooleanInput hasRightColour;

    private double speedOffset = STARTING_SHOOTER_SPEED_OFFSET;
    private boolean isShootWhileMoving = false;

    public enum SystemState implements State<SystemState, Shooter> {
        SHOOTING {
            @Override
            public void acceptShootingValues(Shooter shooter, double shooterSpeed, boolean readyToShoot) {
                shooter.targetVelocityMps.accept(shooterSpeed + shooter.speedOffset);
                shooter.isReadyToBeFed.accept(Math.abs(shooter.targetVelocityMps.get() - shooter.velocityMps.get()) < TARGET_MPS_TOLERANCE && readyToShoot);
            }

            @Override
            public void deconstructState(Shooter shooter) {
                shooter.targetVelocityMps.accept(IDLE_TARGET_MPS);
                shooter.isReadyToBeFed.accept(false);
            }
        },
        LOW_GOAL_SHOOTING { // FIXME Refactor during shoot-while-moving development
            @Override
            public void run(Shooter shooter) {
                shooter.targetVelocityMps.accept(ShooterConstants.LOW_GOAL_SHOOTING_TARGET_MPS + shooter.speedOffset);
                shooter.isReadyToBeFed.accept(Math.abs(shooter.targetVelocityMps.get() - shooter.velocityMps.get()) < TARGET_MPS_TOLERANCE);
            }

            @Override
            public void deconstructState(Shooter shooter) {
                shooter.targetVelocityMps.accept(IDLE_TARGET_MPS);
                shooter.isReadyToBeFed.accept(false);
            }
        },
        IDLE { // FIXME Refactor during shoot-while-moving development
            @Override
            public void acceptShootingValues(Shooter shooter, double shooterSpeed, boolean readyToShoot) {
                shooter.targetVelocityMps.accept(shooterSpeed);
                shooter.isReadyToBeFed.accept(!shooter.hasRightColour.get() && readyToShoot);
            }
        },
        IDLE_LOW { // FIXME Refactor during shoot-while-moving development
            @Override
            public void run(Shooter shooter) {
                shooter.targetVelocityMps.accept(LOW_GOAL_SHOOTING_TARGET_MPS);
                shooter.isReadyToBeFed.accept(!shooter.hasRightColour.get());
            }
        };

        void acceptShootingValues(Shooter shooter, double shooterSpeed, boolean readyToShoot) {}
    }

    public Shooter() {
        super(SystemState.IDLE, SystemState.class);

        targetVelocityMps = new DoublePipe(0);
        isReadyToBeFed = new BooleanPipe(false).setFilter(new ChangedBooleanFilter(false)); // Only update dependents when value changes

        velocityMps = new GettableDoubleInput();
        distanceToHubMetres = new GettableDoubleInput();
        hasRightColour = new GettableBooleanInput();
    }

    @Override
    protected Shooter getSelf() {
        return this;
    }

    @Override
    public void run() {
        super.run();

        Dashboard.getInstance().putNumber("Shooter M/S", velocityMps.get());
        Dashboard.getInstance().putNumber("Shooter Offset M/S", speedOffset);

        // System.out.println(velocityMps.get() + "," + targetVelocityMps.get());
    }

    public void acceptShootingValues(FlaggedDoubleValue value) {
        state.acceptShootingValues(this, value.getValue(), value.getFlag());
    }

    public void shootWhileMoving(boolean activate) {
        isShootWhileMoving = true;
        setState(activate ? SystemState.SHOOTING : SystemState.IDLE);
    }

    public void shootStationary(boolean activate) {
        isShootWhileMoving = false;
        setState(activate ? SystemState.SHOOTING : SystemState.IDLE);
    }

    public void lowGoalShoot(boolean activate) {
        setState(activate ? SystemState.LOW_GOAL_SHOOTING : SystemState.IDLE);
    }

    public void idleLow() {
        setState(SystemState.IDLE_LOW);
    }

    public boolean isStrategyShootWhileMoving() {
        return isShootWhileMoving;
    }

    public DoubleSource getTargetVelocityMpsOutput() {
        return targetVelocityMps;
    }

    public BooleanSource getReadyToBeFedOutput() {
        return isReadyToBeFed;
    }

    public Input<GettableDouble> getVelocityMpsSensor() {
        return velocityMps;
    }

    public Input<GettableDouble> getDistanceToHubMetresSensor() {
        return distanceToHubMetres;
    }

    public Input<GettableBoolean> getHasRightColourSensor() {
        return hasRightColour;
    }

    public void increaseSpeedOffset() {
        speedOffset += SPEED_OFFSET_INCREMENT_MPS;
    }

    public void decreaseSpeedOffset() {
        speedOffset -= SPEED_OFFSET_INCREMENT_MPS;
    }
}
