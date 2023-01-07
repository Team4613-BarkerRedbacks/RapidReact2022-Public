package redbacks.robot.subsystems.drivetrain;

import arachne.lib.dashboard.Dashboard;
import arachne.lib.game.GameState;
import arachne.lib.immutables.Pair;
import arachne.lib.io.Gettable;
import arachne.lib.io.sensors.GettableBooleanInput;
import arachne.lib.io.sensors.GettableInput;
import arachne.lib.io.sensors.Input;
import arachne.lib.logging.ArachneLogger;
import arachne.lib.sequences.Actionable;
import arachne.lib.sequences.Untilable;
import arachne.lib.sequences.actions.Action;
import arachne.lib.states.State;
import arachne.lib.states.StatefulSubsystem;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import redbacks.robot.Constants;
import redbacks.robot.subsystems.aiming.VisionCalculatedPosition;
import redbacks.robot.subsystems.drivetrain.DrivetrainConstants.ModulePosition;

import static redbacks.robot.subsystems.drivetrain.DrivetrainConstants.*;

import java.util.Arrays;
import java.util.function.Function;
import java.util.function.Supplier;

public class Drivetrain extends StatefulSubsystem<Drivetrain.SystemState, Drivetrain> {
    // Modules
    private final SwerveModuleSet<ModulePosition> modules;

    // Sensors
    private final GettableInput<Rotation2d> yaw;

    // Control variables
    private final SwerveDrivePoseEstimator odometry;
    private final SwerveDriveKinematics kinematics;

    private final GettableBooleanInput holdingOneBall;
    private final GettableBooleanInput holdingTwoBalls;

    private Pose2d lastPose;

    enum SystemState implements State<SystemState, Drivetrain> {
        AUTO,
        FIELD_RELATIVE_DRIVER_CONTROLLED {
            @Override
            void acceptDriverInputs(Drivetrain drivetrain, double forward, double left, double rotate) {
                drivetrain.feedModulesFromTargetVelocity(
                    ChassisSpeeds.fromFieldRelativeSpeeds(
                        percentToVelocity(forward),
                        percentToVelocity(left),
                        percentToRotationalVelocity(rotate),
                        drivetrain.odometry.getEstimatedPosition().getRotation()
                    )
                );
            }
        };

        void acceptDriverInputs(Drivetrain drivetrain, double forward, double left, double rotate) {}
    }

    public Drivetrain() {
        super(SystemState.FIELD_RELATIVE_DRIVER_CONTROLLED, SystemState.class);

        this.yaw = new GettableInput<Rotation2d>();

        this.modules = new SwerveModuleSet<ModulePosition>(
            SwerveModule::new,
            ModulePosition.values(),
            ModulePosition::getOffset
        );

        this.kinematics = new SwerveDriveKinematics(modules.getOffsets());
        this.odometry = new SwerveDrivePoseEstimator(
			new Rotation2d(), new Pose2d(), kinematics,
			new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.1, 0.1, 0.1), // State measurement standard deviations. X, Y, theta. 
			new MatBuilder<>(Nat.N1(), Nat.N1()).fill(0.01), // Local measurement standard deviations. Gyro in radians.
			new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.02, 0.02, 0.01), // Global measurement standard deviations. X, Y, theta. Sourced from field localisation such as vision
			Constants.LOOP_PERIOD
		);

        this.holdingOneBall = new GettableBooleanInput();
        this.holdingTwoBalls = new GettableBooleanInput();
    }

    @Override
    protected Drivetrain getSelf() {
        return this;
    }

    @Override
    public void run() {
        super.run();
        updateOdometry();

        Dashboard.getInstance().putBoolean("One Ball", holdingOneBall.get());
        Dashboard.getInstance().putBoolean("Two Ball", holdingTwoBalls.get());
    }

    public void onGameStateChange(GameState gameState) {
        if(gameState == GameState.AUTO) setState(SystemState.AUTO);
        else setState(SystemState.FIELD_RELATIVE_DRIVER_CONTROLLED);
    }

    public void acceptDriverInputs(double forward, double left, double rotate) {
        state.acceptDriverInputs(this, forward, left, rotate);
    }

    private void updateOdometry() {
        SwerveModuleState[] moduleStates = new SwerveModuleState[modules.size()];

        modules.forEach((module, i) -> moduleStates[i] = module.getModuleState());

        try {
            // If the method call does not throw an exception, update the last known pose
            // We store the pose to reset to in case of errors due to timing in updateWithTime()
            lastPose = odometry.updateWithTime(Timer.getFPGATimestamp(), yaw.get(), moduleStates);
        }
        catch(RuntimeException e) {
            ArachneLogger.getInstance().critical("Odometry error thrown");

            // If updateWithTime() throws an exception, reset to the last known position
            odometry.resetPosition(lastPose, yaw.get());
        }

        for(int i = 0; i < modules.size(); i++) {
            SmartDashboard.putNumber("Module angle " + ModulePosition.values()[i], moduleStates[i].angle.getDegrees());
        }
    }

    /**
     * Converts from a Chassis state to individual states for the modules.
     * This is responsible for providing each SwerveModule with it's angle and velocity.
     * It is not field relative but relative to the chassis.
     * @param swerveModules
     * @param kinematics The kinematics object used to calculate the behaviors for each module.
     * @param A ChassisSpeed object with the following parameters:
     * 						forward Metres per second forward
     * 						strafe Metres per second left (negative goes right)
     * 						rotation Rotation in radians per second counterclockwise.
     */
    private void feedModulesFromTargetVelocity(ChassisSpeeds chassisSpeeds) {
        SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(chassisSpeeds);
        modules.forEach((module, i) -> module.driveAt(moduleStates[i]));
    }

    private static double percentToVelocity(double percent) {
        return percent * MAX_VELOCITY_METRES_PER_SEC;
    }

    private static double percentToRotationalVelocity(double percent) {
        return percent * TELEOP_MAX_ROTATIONAL_VELOCITY_RADIANS_PER_SEC;
    }

    public Pose2d getPosition() {
        return odometry.getEstimatedPosition();
    }

    public double getDistanceToHub() {
        return getPosition().getTranslation().getNorm();
    }

    /**
     * @param posX The position in metres along long end of the field (The centre is 0, your side is negative)
     * @param posY The position in metres along short end of field (The centre is 0, the right is negative)
     * @param angle The angle in degrees of the robot (Facing the opposing alliance station is 0)
     */
    public void setPosition(double posX, double posY, Rotation2d angle) {
        odometry.resetPosition(new Pose2d(posX, posY, angle), yaw.get());
    }

    public void setPosition(Translation2d pos, Rotation2d angle) {
        setPosition(pos.getX(), pos.getY(), angle);
    }

    public void setYPosition(double posY) {
        setPosition(odometry.getEstimatedPosition().getX(), posY, odometry.getEstimatedPosition().getRotation());
    }

    public void setHeading(Rotation2d angle) {
        Pose2d currentPosition = getPosition();
        setPosition(currentPosition.getX(), currentPosition.getY(), angle);
    }

    public void updatePositionFromVision(VisionCalculatedPosition result) {
        odometry.addVisionMeasurement(result.position, Timer.getFPGATimestamp() - result.latencySeconds);
    }

    public Pair<Double, ChassisSpeeds> getHubRelativeChassisSpeeds() {
        SwerveModuleState[] moduleStates = new SwerveModuleState[modules.size()];
        modules.forEach((module, i) -> moduleStates[i] = module.getModuleState());

        ChassisSpeeds chassisSpeeds = kinematics.toChassisSpeeds(moduleStates);

        Pose2d fieldRelativePosition = getPosition();
 
        Rotation2d robotToHubDirection = new Rotation2d(
            -fieldRelativePosition.getTranslation().getX(),
            -fieldRelativePosition.getTranslation().getY()
        );

        Rotation2d robotDirection = fieldRelativePosition.getRotation();
        Translation2d fieldRelativeVelocities = new Translation2d(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond).rotateBy(robotDirection.minus(robotToHubDirection));

        return new Pair<Double, ChassisSpeeds>(
            fieldRelativePosition.getTranslation().getNorm(), 
            new ChassisSpeeds(fieldRelativeVelocities.getX(), fieldRelativeVelocities.getY(), chassisSpeeds.omegaRadiansPerSecond)
        );
    }

    // ----------------------------------------
    // IO bindings
    // ----------------------------------------

    public SwerveModule getModule(ModulePosition position) {
        return modules.getModule(position);
    }

    public Input<Gettable<Rotation2d>> getYawSensor() {
        return yaw;
    }

    public GettableBooleanInput getHoldingOneBallSensor() {
        return holdingOneBall;
    }

    public GettableBooleanInput getHoldingTwoBallsSensor() {
        return holdingTwoBalls;
    }

    public double getHubRelativeYawDegrees() {
        Pose2d fieldRelativePosition = getPosition();

        Rotation2d robotToHubDirection = new Rotation2d(
            -fieldRelativePosition.getTranslation().getX(),
            -fieldRelativePosition.getTranslation().getY()
        );

        Rotation2d robotDirection = fieldRelativePosition.getRotation();

        return robotDirection.minus(robotToHubDirection).getDegrees();
    }

    // ----------------------------------------
    // Actionables
    // ----------------------------------------

    public Untilable doMoveTo(Pose2d target, double maxVelocityMPS) {
        return doMoveTo(target.getTranslation(), (position) -> target.getRotation(), maxVelocityMPS);
    }

    public Untilable doMoveTo(Translation2d target, Rotation2d rotation, double maxVelocityMPS) {
        return doMoveTo(target, (position) -> rotation, maxVelocityMPS);
    }

    public Untilable doMoveTo(Pose2d target) {
        return doMoveTo(target.getTranslation(), (position) -> target.getRotation());
    }

    public Untilable doMoveTo(Translation2d target, Rotation2d rotation) {
        return doMoveTo(target, (position) -> rotation);
    }

    public Untilable doMoveToTargetingHub(double x, double y) {
        return doMoveToTargetingHub(new Translation2d(x, y));
    }

    public Untilable doMoveToTargetingHub(Translation2d target) {
        return doMoveTo(target, (position) -> new Rotation2d(-position.getX(), -position.getY()));
    }

    public Untilable doMoveTo(Translation2d target, Function<Translation2d, Rotation2d> targetHeadingFromLocation) {
        return doMoveTo(target, targetHeadingFromLocation, MAX_VELOCITY_METRES_PER_SEC);
    }

    public Untilable doMoveTo(Translation2d target, Function<Translation2d, Rotation2d> targetHeadingFromLocation, double maxVelocityMPS) {
        return (host, conditionModifier) -> new Action(host, conditionModifier) {
            HolonomicDriveController controller;
            Trajectory trajectory;
            double startTime;

            @Override
            protected void initialize() {
				setState(SystemState.AUTO);

                ProfiledPIDController rotationController = new ProfiledPIDController(ROTATIONAL_KP, ROTATIONAL_KI, ROTATIONAL_KD, AUTO_ROTATION_MOTION_CONSTRAINTS);
                rotationController.enableContinuousInput(-Math.PI, Math.PI);

                controller = new HolonomicDriveController(
                    new PIDController(LINEAR_KP, LINEAR_KI, LINEAR_KD),
                    new PIDController(LINEAR_KP, LINEAR_KI, LINEAR_KD),
                    rotationController
                );

                Translation2d currentLocation = odometry.getEstimatedPosition().getTranslation();
                Translation2d targetLocation = target;
                Translation2d delta = target.minus(currentLocation);
                Rotation2d deltaDirection = new Rotation2d(delta.getX(), delta.getY());

                trajectory = TrajectoryGenerator.generateTrajectory(
                    new Pose2d(currentLocation, deltaDirection),
                    Arrays.asList(),
                    new Pose2d(targetLocation, deltaDirection),
                    new TrajectoryConfig(maxVelocityMPS, MAX_ACCELERATION_METRES_PER_SECOND_SQUARED)
                );

                startTime = Timer.getFPGATimestamp();
            }

            @Override
            protected void execute() {
                Pose2d currentPosition = odometry.getEstimatedPosition();

                feedModulesFromTargetVelocity(controller.calculate(
                    currentPosition,
                    trajectory.sample(Timer.getFPGATimestamp() - startTime),
                    targetHeadingFromLocation.apply(currentPosition.getTranslation())
                ));
            }

			@Override
			protected void end() {
				feedModulesFromTargetVelocity(new ChassisSpeeds(0, 0, 0));
			}

			@Override
			protected boolean isFinished() {
                Pose2d currentPosition = odometry.getEstimatedPosition();

				return target.minus(currentPosition.getTranslation()).getNorm() < POSITION_TOLERANCE_METRES
                    && Math.abs(targetHeadingFromLocation.apply(target).minus(currentPosition.getRotation()).getRadians()) < ROTATION_TOLERANCE_RADIANS;
			}
        };
    }

    public Untilable doMoveTo(Gettable<Translation2d> relativeCoordinates, Translation2d offset, Rotation2d rotation, double maxVelocityMPS) {
        return (host, conditionModifier) -> new Action(host, conditionModifier) {
            HolonomicDriveController controller;
            Trajectory trajectory;
            double startTime;
            Function<Translation2d, Rotation2d> targetHeadingFromLocation = (position) -> rotation;
            Translation2d target;

            @Override
            protected void initialize() {
				setState(SystemState.AUTO);

                target = relativeCoordinates.get().plus(offset);

                ProfiledPIDController rotationController = new ProfiledPIDController(ROTATIONAL_KP, ROTATIONAL_KI, ROTATIONAL_KD, AUTO_ROTATION_MOTION_CONSTRAINTS);
                rotationController.enableContinuousInput(-Math.PI, Math.PI);

                controller = new HolonomicDriveController(
                    new PIDController(LINEAR_KP, LINEAR_KI, LINEAR_KD),
                    new PIDController(LINEAR_KP, LINEAR_KI, LINEAR_KD),
                    rotationController
                );

                Translation2d currentLocation = odometry.getEstimatedPosition().getTranslation();
                Translation2d targetLocation = target;
                Translation2d delta = target.minus(currentLocation);
                Rotation2d deltaDirection = new Rotation2d(delta.getX(), delta.getY());

                trajectory = TrajectoryGenerator.generateTrajectory(
                    new Pose2d(currentLocation, deltaDirection),
                    Arrays.asList(),
                    new Pose2d(targetLocation, deltaDirection),
                    new TrajectoryConfig(maxVelocityMPS, MAX_ACCELERATION_METRES_PER_SECOND_SQUARED)
                );

                startTime = Timer.getFPGATimestamp();
            }

            @Override
            protected void execute() {
                Pose2d currentPosition = odometry.getEstimatedPosition();

                feedModulesFromTargetVelocity(controller.calculate(
                    currentPosition,
                    trajectory.sample(Timer.getFPGATimestamp() - startTime),
                    targetHeadingFromLocation.apply(currentPosition.getTranslation())
                ));
            }

			@Override
			protected void end() {
				feedModulesFromTargetVelocity(new ChassisSpeeds(0, 0, 0));
			}

			@Override
			protected boolean isFinished() {
                Pose2d currentPosition = odometry.getEstimatedPosition();

				return target.minus(currentPosition.getTranslation()).getNorm() < POSITION_TOLERANCE_METRES
                    && Math.abs(targetHeadingFromLocation.apply(target).minus(currentPosition.getRotation()).getRadians()) < ROTATION_TOLERANCE_RADIANS;
			}
        };
    }

    public Untilable doMoveRight(double moveForSeconds, double velocityMPS) {
        return (host, conditionModifier) -> new Action(host, conditionModifier) {
            double startTime;

            @Override
            protected void initialize() {
				setState(SystemState.AUTO);

                startTime = Timer.getFPGATimestamp();
            }

            @Override
            protected void execute() {
                feedModulesFromTargetVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(0, -velocityMPS, 0, odometry.getEstimatedPosition().getRotation()));
            }

			@Override
			protected void end() {
				feedModulesFromTargetVelocity(new ChassisSpeeds(0, 0, 0));
			}

			@Override
			protected boolean isFinished() {
                double currentTime = Timer.getFPGATimestamp();

				return currentTime - startTime > moveForSeconds;
			}
        };
    }

    public Untilable doSweepToNextBallDown(Gettable<Translation2d> relativeCoordinates, Translation2d offset, Rotation2d rotation, double maxVelocityMPS) {
        return (host, conditionModifier) -> new Action(host, conditionModifier) {
            HolonomicDriveController controller;
            Trajectory trajectory;
            Translation2d target;
            Function<Translation2d, Rotation2d> targetHeadingFromLocation = (position) -> rotation;
            double startTime;

            @Override
            protected void initialize() {
				setState(SystemState.AUTO);

                target = relativeCoordinates.get().plus(offset);

                ProfiledPIDController rotationController = new ProfiledPIDController(ROTATIONAL_KP, ROTATIONAL_KI, ROTATIONAL_KD, AUTO_ROTATION_MOTION_CONSTRAINTS);
                rotationController.enableContinuousInput(-Math.PI, Math.PI);

                controller = new HolonomicDriveController(
                    new PIDController(LINEAR_KP, LINEAR_KI, LINEAR_KD),
                    new PIDController(LINEAR_KP, LINEAR_KI, LINEAR_KD),
                    rotationController
                );

                Translation2d currentLocation = odometry.getEstimatedPosition().getTranslation();
                Translation2d targetLocation = target;
                Translation2d delta = target.minus(currentLocation);
                Rotation2d deltaDirection = new Rotation2d(delta.getX(), delta.getY());

                trajectory = TrajectoryGenerator.generateTrajectory(
                    new Pose2d(currentLocation, deltaDirection),
                    Arrays.asList(),
                    new Pose2d(targetLocation, deltaDirection),
                    new TrajectoryConfig(maxVelocityMPS, MAX_ACCELERATION_METRES_PER_SECOND_SQUARED)
                );

                startTime = Timer.getFPGATimestamp();
            }

            @Override
            protected void execute() {
                Pose2d currentPosition = odometry.getEstimatedPosition();

                ChassisSpeeds chassisSpeeds = controller.calculate(
                    currentPosition,
                    trajectory.sample(Timer.getFPGATimestamp() - startTime),
                    targetHeadingFromLocation.apply(currentPosition.getTranslation())
                );
                chassisSpeeds.vyMetersPerSecond -= 1;
                feedModulesFromTargetVelocity(chassisSpeeds);
            }

			@Override
			protected void end() {
				feedModulesFromTargetVelocity(new ChassisSpeeds(0, 0, 0));
			}

			@Override
			protected boolean isFinished() {
                if(holdingOneBall.get() || holdingTwoBalls.get()) return true;

                Pose2d currentPosition = odometry.getEstimatedPosition();
				return target.minus(currentPosition.getTranslation()).getNorm() < POSITION_TOLERANCE_METRES
                    && Math.abs(targetHeadingFromLocation.apply(target).minus(currentPosition.getRotation()).getRadians()) < ROTATION_TOLERANCE_RADIANS;
			}
        };
    }

    public Untilable doSweepToNextBallUp(Gettable<Translation2d> relativeCoordinates, Translation2d offset, Rotation2d rotation, double maxVelocityMPS) {
        return (host, conditionModifier) -> new Action(host, conditionModifier) {
            HolonomicDriveController controller;
            Trajectory trajectory;
            Translation2d target;
            Function<Translation2d, Rotation2d> targetHeadingFromLocation = (position) -> rotation;
            double startTime;

            @Override
            protected void initialize() {
				setState(SystemState.AUTO);

                target = relativeCoordinates.get().plus(offset);

                ProfiledPIDController rotationController = new ProfiledPIDController(ROTATIONAL_KP, ROTATIONAL_KI, ROTATIONAL_KD, AUTO_ROTATION_MOTION_CONSTRAINTS);
                rotationController.enableContinuousInput(-Math.PI, Math.PI);

                controller = new HolonomicDriveController(
                    new PIDController(LINEAR_KP, LINEAR_KI, LINEAR_KD),
                    new PIDController(LINEAR_KP, LINEAR_KI, LINEAR_KD),
                    rotationController
                );

                Translation2d currentLocation = odometry.getEstimatedPosition().getTranslation();
                Translation2d targetLocation = target;
                Translation2d delta = target.minus(currentLocation);
                Rotation2d deltaDirection = new Rotation2d(delta.getX(), delta.getY());

                trajectory = TrajectoryGenerator.generateTrajectory(
                    new Pose2d(currentLocation, deltaDirection),
                    Arrays.asList(),
                    new Pose2d(targetLocation, deltaDirection),
                    new TrajectoryConfig(maxVelocityMPS, MAX_ACCELERATION_METRES_PER_SECOND_SQUARED)
                );

                startTime = Timer.getFPGATimestamp();
            }

            @Override
            protected void execute() {
                Pose2d currentPosition = odometry.getEstimatedPosition();

                ChassisSpeeds chassisSpeeds = controller.calculate(
                    currentPosition,
                    trajectory.sample(Timer.getFPGATimestamp() - startTime),
                    targetHeadingFromLocation.apply(currentPosition.getTranslation())
                );
                chassisSpeeds.vyMetersPerSecond += 1;
                feedModulesFromTargetVelocity(chassisSpeeds);
            }

			@Override
			protected void end() {
				feedModulesFromTargetVelocity(new ChassisSpeeds(maxVelocityMPS, 0, 0));
			}

			@Override
			protected boolean isFinished() {
                if(holdingOneBall.get() || holdingTwoBalls.get()) return true;

                Pose2d currentPosition = odometry.getEstimatedPosition();
				return target.minus(currentPosition.getTranslation()).getNorm() < POSITION_TOLERANCE_METRES
                    && Math.abs(targetHeadingFromLocation.apply(target).minus(currentPosition.getRotation()).getRadians()) < ROTATION_TOLERANCE_RADIANS;
			}
        };
    }

    public Actionable doTurnToHub() {
        return doTurnTo(() -> {
            Translation2d currentLocation = odometry.getEstimatedPosition().getTranslation();
            return new Rotation2d(-currentLocation.getX(), -currentLocation.getY());
        });
    }

    public Actionable doTurnTo(double angleDegrees) {
		return doTurnTo(Rotation2d.fromDegrees(angleDegrees));
	}

    public Actionable doTurnTo(Rotation2d angle) {
		return doTurnTo(() -> angle);
	}

    public Actionable doTurnTo(Supplier<Rotation2d> targetSupplier) {
		return (host) -> new Action(host) {
            ProfiledPIDController controller;
            Rotation2d target;

            @Override
            protected void initialize() {
				setState(SystemState.AUTO);
                target = targetSupplier.get();
                controller = new ProfiledPIDController(ROTATIONAL_KP, ROTATIONAL_KI, ROTATIONAL_KD, AUTO_ROTATION_MOTION_CONSTRAINTS);
                controller.enableContinuousInput(-Math.PI, Math.PI);
                controller.setGoal(target.getRadians());
                controller.setTolerance(ROTATION_TOLERANCE_RADIANS);
            }

            @Override
            protected void execute() {
                feedModulesFromTargetVelocity(new ChassisSpeeds(
                    0,
                    0,
                    controller.calculate(odometry.getEstimatedPosition().getRotation().getRadians())
                ));
            }

			@Override
			protected void end() {
				feedModulesFromTargetVelocity(new ChassisSpeeds(0, 0, 0));
			}

			@Override
			protected boolean isFinished() {
				return controller.atGoal();
			}
        };
	}
}
