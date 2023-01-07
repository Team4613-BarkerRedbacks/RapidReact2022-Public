package redbacks.robot.subsystems.ballhandling;

import arachne.lib.dashboard.Dashboard;
import arachne.lib.io.Gettable;
import arachne.lib.io.GettableBoolean;
import arachne.lib.io.GettableDouble;
import arachne.lib.io.sensors.GettableBooleanInput;
import arachne.lib.io.sensors.GettableDoubleInput;
import arachne.lib.io.sensors.GettableInput;
import arachne.lib.io.sensors.Input;
import arachne.lib.pipeline.BooleanPipe;
import arachne.lib.pipeline.BooleanSource;
import arachne.lib.pipeline.DoublePipe;
import arachne.lib.pipeline.DoubleSource;
import arachne.lib.pipeline.filters.ChangedBooleanFilter;
import arachne.lib.sequences.Actionable;
import arachne.lib.sequences.actions.Action;
import arachne.lib.sequences.actions.HostAction;
import arachne.lib.states.State;
import arachne.lib.states.StatefulSubsystem;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import redbacks.robot.Robot;

import static redbacks.robot.subsystems.ballhandling.BallHandlingConstants.*;
import static arachne.lib.sequences.Actionable.*;

public class BallHandling extends StatefulSubsystem<BallHandling.SystemState, BallHandling> {
    private final DoublePipe intake, tower;
    private final BooleanPipe extendIntake;
    private final GettableBooleanInput ballAtTop, ballAtBottom, manuallyRetractIntake;
    private final BooleanPipe rumbleController;
    private final GettableDoubleInput positionTicks;

    private final GettableInput<BallColour> currentBallColour;
    private BallColour lastSeenColour = BallColour.NONE;
    private BallColour targetColour = BallColour.NONE;
    private double positionOfLastSeenColour = Double.NaN;

    private final ChangedBooleanFilter hasBallAtTopChanged = new ChangedBooleanFilter(false);
    private final ChangedBooleanFilter hasFullnessChanged = new ChangedBooleanFilter(false);

    private final Actionable doOneBallRumbleController, doTwoBallRumbleController;

    private boolean hasBeenManuallyRetracted = false;

    private final Robot robot;

    public BallHandling(Robot robot) {
        super(SystemState.INTAKING, SystemState.class);

        intake = new DoublePipe(0);
        tower = new DoublePipe(0);

        this.robot = robot;

        extendIntake = new BooleanPipe(false);
        rumbleController = new BooleanPipe(false);

        ballAtTop = new GettableBooleanInput();
        ballAtBottom = new GettableBooleanInput();
        manuallyRetractIntake = new GettableBooleanInput();

        positionTicks = new GettableDoubleInput();

        currentBallColour = new GettableInput<BallColour>();

        doOneBallRumbleController = SEQUENCE(
            DO(() -> rumbleController.accept(true)),
            WAIT(300),
            DO(() -> rumbleController.accept(false))
        );

        doTwoBallRumbleController = SEQUENCE(
            doOneBallRumbleController,
            WAIT(100),
            doOneBallRumbleController
        );
    }
    @Override
    protected BallHandling getSelf() {
        return this;
    }

    public enum SystemState implements State<SystemState, BallHandling> {
        INTAKING { // includes indexing
            @Override
            public void run(BallHandling ballHandling) {
                boolean extendIntake = false;
                if(ballHandling.ballAtTop.get()) {
                    ballHandling.tower.accept(0);

                    extendIntake = manageIntake(ballHandling);
                }
                else {
                    ballHandling.intake.accept(INTAKE_IN_POWER);
                    ballHandling.tower.accept(TOWER_IN_POWER);
                    extendIntake = true;
                }

                ballHandling.extendIntake.accept(ballHandling.manuallyRetractIntake.get() ? false : extendIntake);
            }

            private boolean manageIntake(BallHandling ballHandling) {
                boolean extendIntake = false;
                if(ballHandling.ballAtBottom.get()) {
                    ballHandling.intake.accept(0);
                    extendIntake = false;
                }
                else {
                    ballHandling.intake.accept(INTAKE_IN_POWER);
                    extendIntake = true;
                }
                return extendIntake;
            }
        },
        OUTTAKING {
            @Override
            public void constructState(BallHandling ballHandling) {
                ballHandling.extendIntake.accept(true);
                ballHandling.intake.accept(INTAKE_OUT_POWER);
                ballHandling.tower.accept(TOWER_OUT_POWER);
            }
        },
        SHOOTING {
            @Override
            public void constructState(BallHandling ballHandling) {
                ballHandling.hasBeenManuallyRetracted = ballHandling.manuallyRetractIntake.get();
            }

            @Override
            public Action createStateAction(BallHandling ballHandling) {
                return REPEAT(
                    SEQUENCE(
                        DO(() -> ballHandling.tower.accept(TOWER_FEED_POWER)),
                        DO(() -> System.err.println(
                            "Distance: " + ballHandling.robot.drivetrain.getDistanceToHub()
                            + ", Velocity: " + ballHandling.robot.shooter.getTargetVelocityMpsOutput().get()
                            + ", Pitch: " + ballHandling.robot.turret.getTargetPitch().get() 
                            + ", Match Time: " + DriverStation.getMatchTime()
                        )),
                        WAIT().UNSAFE_UNTIL(ballHandling.ballAtTop),
                        WAIT().UNTIL(() -> {
                            if(!ballHandling.ballAtBottom.get()) return (unused) -> false;
                            else {
                                final double targetPosition = ballHandling.positionTicks.get() + FEED_FIRST_BALL_DISTANCE;
                                return (unused) -> ballHandling.positionTicks.get() > targetPosition;
                            }
                        }),
                        DO(() -> ballHandling.tower.accept(0)),
                        WAIT((long)(SHOOT_WAIT_SECONDS * 1000))
                    )
                ).asAction(null);
            }

            @Override
            public void run(BallHandling ballHandling) {
                if(ballHandling.manuallyRetractIntake.get()) {
                    ballHandling.hasBeenManuallyRetracted = true;
                    ballHandling.extendIntake.accept(false);
                }
                else if(ballHandling.hasBeenManuallyRetracted) {
                    ballHandling.extendIntake.accept(true);
                }
            }
        },
        MANUALLY_INTAKING {
            @Override
            public void constructState(BallHandling ballHandling) {
                ballHandling.extendIntake.accept(!ballHandling.manuallyRetractIntake.get());
                ballHandling.intake.accept(INTAKE_IN_POWER);
                ballHandling.tower.accept(TOWER_IN_POWER);
            }
        },
        STOPPED {
            @Override
            public void constructState(BallHandling ballHandling) {
                ballHandling.extendIntake.accept(false);
                ballHandling.intake.accept(0);
                ballHandling.tower.accept(0);
            }
        },
        ACTIONABLE_CONTROL;

        private static void manageIntake(BallHandling ballHandling) {
            if(ballHandling.ballAtBottom.get()) {
                ballHandling.intake.accept(0.2);
                ballHandling.extendIntake.accept(false);
            }
            else {
                ballHandling.intake.accept(INTAKE_IN_POWER);
                ballHandling.extendIntake.accept(true);
            }
        }
    }

    @Override
    public void run() {
        super.run();

        Dashboard.getInstance().putBoolean("Ball at top", ballAtTop.get());
        Dashboard.getInstance().putBoolean("Ball at bottom", ballAtBottom.get());

        /**
         * We want to read a new colour when we see it,
         * and default back to no colour when we're sure we've fired that ball
         */
        BallColour currentColour = currentBallColour.get();

        if(currentColour != BallColour.NONE) {
            lastSeenColour = currentColour;
            positionOfLastSeenColour = positionTicks.get();
        }
        else if(positionTicks.get() > positionOfLastSeenColour + FEED_FIRST_BALL_DISTANCE) {
            lastSeenColour = currentColour;
        }

        targetColour = DriverStation.getAlliance() == Alliance.Red ? BallColour.RED : BallColour.BLUE; // TODO Provide dashboard toggle

        boolean isBallAtTop = ballAtTop.get();
        boolean isFull = isFull();

        boolean rumbleOneBall = false;
        boolean rumbleFull = false;

        if(!isBallAtTop && !isFull) rumbleController.accept(false);
        if(hasBallAtTopChanged.test(isBallAtTop)) {
            if(isBallAtTop) rumbleOneBall = true;
        }
        if(hasFullnessChanged.test(isFull)) {
            if(isFull) rumbleFull = true;
        }

        if(rumbleFull) internalActionScheduler.add(doTwoBallRumbleController);
        else if(rumbleOneBall) internalActionScheduler.add(doOneBallRumbleController);
    }

    public Actionable doFeedOneBall() {
        return SEQUENCE(
            DO(() -> System.err.println(
                "Distance: " + robot.drivetrain.getDistanceToHub()
                + ", Velocity: " + robot.shooter.getTargetVelocityMpsOutput().get()
                + ", Pitch: " + robot.turret.getTargetPitch().get() 
                + ", Match Time: " + DriverStation.getMatchTime()
            )),
            (host) -> feedOneBall(host)
        );
    }

    public Actionable doFeedTwoBalls(double delaySeconds) {
        return SEQUENCE(
            doFeedOneBall(),
            WAIT((long)(delaySeconds * 1000)),
            doFeedOneBall()
        );
    }

    private Action feedOneBall(HostAction host) {
        return new Action(host) {
            double targetPosition;
            boolean runUntilBall;
            boolean hasSeenBall;

            @Override
            protected void initialize() {
                setState(SystemState.ACTIONABLE_CONTROL);
                targetPosition = positionTicks.get() + FEED_FULL_TOWER_DISTANCE;
                tower.accept(TOWER_FEED_POWER);
            }

            @Override
            protected void execute() {
                if(ballAtBottom.get()) {
                    runUntilBall = true;
                }

                if(ballAtTop.get() && !hasSeenBall) {
                    hasSeenBall = true;
                    targetPosition = positionTicks.get() + (ballAtBottom.get() ? FEED_FIRST_BALL_DISTANCE : FEED_ONLY_BALL_DISTANCE);
                }

                SystemState.manageIntake(BallHandling.this);
            }

            @Override
            protected void end() {
                setState(SystemState.INTAKING);
            }

            @Override
            protected boolean isFinished() {
                return positionTicks.get() > targetPosition && (hasSeenBall || !runUntilBall);
            }
        };
    }

    public void intake() {
        setState(SystemState.INTAKING);
    }

    public void outtake() {
        setState(SystemState.OUTTAKING);
    }

    public void shoot() {
        setState(SystemState.SHOOTING);
    }

    public void stop() {
        setState(SystemState.STOPPED);
    }

    public void intakeManually(boolean activate) {
        setState(activate ? SystemState.MANUALLY_INTAKING : SystemState.STOPPED);
    }

    public boolean hasRightBall() {
        return targetColour == BallColour.NONE
            || lastSeenColour == BallColour.NONE
            || targetColour == lastSeenColour;
    }

    public DoubleSource getIntakeOutput() {
        return intake;
    }

    public DoubleSource getTowerOutput() {
        return tower;
    }

    public BooleanSource getExtendIntakeOutput() {
        return extendIntake;
    }

    public BooleanSource getRumbleControllerOutput() {
        return rumbleController;
    }

    public Input<GettableBoolean> getBallAtTopSensor() {
        return ballAtTop;
    }

    public Input<GettableBoolean> getBallAtBottomSensor() {
        return ballAtBottom;
    }

    public Input<GettableBoolean> getManuallyRetractIntake() {
        return manuallyRetractIntake;
    }

    public boolean isEmpty() {
        return !ballAtTop.get() && !ballAtBottom.get();
    }

    public boolean isHoldingOneBall() {
        return ballAtTop.get() || ballAtBottom.get();
    }

    public boolean isFull() {
        return ballAtTop.get() && ballAtBottom.get();
    }

    public Input<GettableDouble> getPositionTicksSensor() {
        return positionTicks;
    }

    public Input<Gettable<BallColour>> getBallColourSensor() {
        return currentBallColour;
    }

    public int getNumberOfBalls() {
        if(isEmpty()) return 0;
        if(isHoldingOneBall()) return 1;
        if(isFull()) return 2;

        return -1; // something has gone wrong
    }
}
