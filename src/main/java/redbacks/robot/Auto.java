package redbacks.robot;

import arachne.lib.io.Gettable;
import arachne.lib.sequences.Actionable;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

import static arachne.lib.sequences.Actionable.*;

import java.util.function.Function;

public enum Auto implements Function<Robot, Actionable> {
	DO_NOTHING(new Pose2d(-2, 0, new Rotation2d()), (robot) ->
		DO_NOTHING()
	),
	TWO_BALL_LEFT(
		new Pose2d(
			FieldLocation.LEFT_SIDE_STARTING_POSITION.getCoordinates(),
			Rotation2d.fromDegrees(-39)
		), (robot) -> {
		final int BALL5_PATIENCE = 2000; // this is the second ball (including cargo)       

		return SEQUENCE(
			DO(robot.aiming::pauseOdometry),
			// Pick up BALL5
			DO(() -> robot.drivetrain.setPosition(FieldLocation.LEFT_SIDE_STARTING_POSITION.getCoordinates(), Rotation2d.fromDegrees(-39))),
			WAIT(1000),
			robot.drivetrain
				.doMoveTo(FieldLocation.BALL5_RED.getCoordinates().plus(new Translation2d(0.2, -0.2)), Rotation2d.fromDegrees(-15))
				.UNSAFE_UNTIL((isAtTarget) -> isAtTarget || robot.ballHandlingHardware.ballAtBottom.get()),
			DO(robot.aiming::resume),
			robot.drivetrain
			.doMoveTo(FieldLocation.BALL5_RED.getCoordinates().plus(new Translation2d(-1, -0.2)), Rotation2d.fromDegrees(-15)),
			WAIT(500),
			WAIT(BALL5_PATIENCE).UNSAFE_UNTIL((hasWaited) -> hasWaited || robot.ballHandlingHardware.ballAtBottom.get()),

			// Shoot cargo and BALL5
			DO(() -> robot.shooter.shootStationary(true))
		);
	}),
	FOUR_BALL_LEFT(
		new Pose2d(
			FieldLocation.LEFT_SIDE_STARTING_POSITION.getCoordinates(),
			Rotation2d.fromDegrees(-39)
		), (robot) -> {
		final int BALL5_PATIENCE = 2000; // this is the second ball (including cargo)
        final int BALL_TERMINAL_PATIENCE = 300;
        final int BALL_HUMAN_PLAYER_PATIENCE = 1500;

		return SEQUENCE(
			DO(robot.aiming::pauseOdometry),
			DO(() -> robot.shooter.shootStationary(false)),
			// Pick up BALL5
			DO(() -> robot.drivetrain.setPosition(FieldLocation.LEFT_SIDE_STARTING_POSITION.getCoordinates(), Rotation2d.fromDegrees(-39))),
			robot.drivetrain
				.doMoveTo(FieldLocation.BALL5_RED.getCoordinates().plus(new Translation2d(0.2, -0.2)), Rotation2d.fromDegrees(-15))
				.UNSAFE_UNTIL((isAtTarget) -> isAtTarget || robot.ballHandlingHardware.ballAtBottom.get()),
			DO(robot.aiming::resume),
			WAIT(500),
			WAIT(BALL5_PATIENCE).UNSAFE_UNTIL((hasWaited) -> hasWaited || robot.ballHandlingHardware.ballAtBottom.get()),

			// Shoot cargo and BALL5
			robot.ballHandling.doFeedTwoBalls(0.5),

			// Ball terminal and ball human player
			// DO(robot.aiming::pauseOdometry),
			robot.drivetrain
				.doMoveTo(FieldLocation.BALL_TERMINAL_RED.getCoordinates().plus(new Translation2d(0.7, -0.7)), Rotation2d.fromDegrees(35))
				.UNSAFE_UNTIL((isAtTarget) -> isAtTarget || robot.ballHandlingHardware.ballAtBottom.get()),
			WAIT(BALL_TERMINAL_PATIENCE).UNSAFE_UNTIL((hasWaited) -> hasWaited || robot.ballHandlingHardware.ballAtBottom.get()),
			robot.drivetrain
				.doMoveTo(FieldLocation.BALL_TERMINAL_RED.getCoordinates().plus(new Translation2d(0.7, 0.4)), Rotation2d.fromDegrees(45)),
			WAIT(BALL_HUMAN_PLAYER_PATIENCE).UNSAFE_UNTIL((hasWaited) -> hasWaited || robot.ballHandling.isFull()),
			robot.drivetrain
				.doMoveTo(FieldLocation.BALL_TERMINAL_RED.getCoordinates().plus(new Translation2d(3, 0.5)), Rotation2d.fromDegrees(45)),
			// DO(robot.aiming::resume),
			WAIT(500),
			robot.ballHandling.doFeedTwoBalls(0.8),

			DO(() -> System.out.println("Finished auto"))
		);
	}),
	// FOUR_BALL_LEFT((robot) -> {
	// 	final int BALL5_PATIENCE = 2000; // this is the second ball (including cargo)
	// 	final int SHOT1_BUFFER = 1500;

	// 	return SEQUENCE(

	// 		// Pick up BALL5
	// 		DO(() -> robot.drivetrain.setPosition(FieldLocation.LEFT_SIDE_STARTING_POSITION.getCoordinates(), Rotation2d.fromDegrees(-63))),
	// 		// robot.drivetrain.doMoveTo(FieldLocation.BALL5_RED.getCoordinates().plus(new Translation2d(0.8, -0.55)), Rotation2d.fromDegrees(-15)),
	// 		robot.drivetrain
	// 			.doMoveTo(FieldLocation.BALL5_RED.getCoordinates().plus(new Translation2d(0.4, 0.1)), Rotation2d.fromDegrees(-15))
	// 			.UNSAFE_UNTIL((isAtTarget) -> isAtTarget || robot.ballHandlingHardware.ballAtBottom.get()),
	// 		WAIT(BALL5_PATIENCE).UNSAFE_UNTIL((hasWaited) -> hasWaited || robot.ballHandlingHardware.ballAtBottom.get()),

	// 		// Shoot cargo and BALL5
	// 		DO(() -> robot.shooter.shoot(true)),
	// 		WAIT().UNSAFE_UNTIL(robot.ballHandling::isEmpty),
	// 		WAIT(SHOT1_BUFFER),
	// 		DO(() -> robot.shooter.shoot(false)),

	// 		// Pick up terminal and human player ball
	// 		robot.drivetrain.doMoveTo(FieldLocation.BALL5_RED.getCoordinates().plus(new Translation2d(0.15, -1)), Rotation2d.fromDegrees(45)),
	// 		// robot.drivetrain.doMoveTo(FieldLocation.BALL_TERMINAL_RED.getCoordinates().plus(new Translation2d(1.2, 0.8)), Rotation2d.fromDegrees(45)),
	// 		robot.drivetrain
	// 			.doMoveTo(FieldLocation.BALL_TERMINAL_RED.getCoordinates().plus(new Translation2d(0.75, -0.3)), Rotation2d.fromDegrees(45))
	// 			.UNSAFE_UNTIL((isAtTarget) -> isAtTarget || robot.ballHandlingHardware.ballAtBottom.get()),
	// 		WAIT(1000),
	// 		WAIT(2000).UNSAFE_UNTIL((hasWaited) -> hasWaited || robot.ballHandling.isFull()),
	// 		robot.drivetrain.doMoveTo(FieldLocation.BALL5_RED.getCoordinates().plus(new Translation2d(-0.8, -0.8)), Rotation2d.fromDegrees(45)),
	// 		WAIT(300),
	// 		DO(() -> robot.shooter.shoot(true)),

	// 		DO(() -> System.out.println("Finished auto"))
	// 	);
	// }),
	FIVE_BALL_RIGHT(
		new Pose2d(
			FieldLocation.RIGHT_SIDE_STARTING_POSITION.getCoordinates(),
			Rotation2d.fromDegrees(90)
		), (robot) -> {
        final int BALL2_PATIENCE = 1000;
        final int BALL3_PATIENCE = 1000;
        final int BALL_TERMINAL_PATIENCE = 300;
        final int BALL_HUMAN_PLAYER_PATIENCE = 1500;

		return SEQUENCE(
			// Cargo and ball 2
			DO(robot.aiming::pauseOdometry), // the odometry will not update but the turret will
			DO(() -> robot.shooter.shootStationary(false)), // idle at speed and manually feed
			DO(robot.ballHandling::intake),
			DO(() -> robot.drivetrain.setPosition(FieldLocation.RIGHT_SIDE_STARTING_POSITION.getCoordinates(), Rotation2d.fromDegrees(90))),
			robot.drivetrain
				.doMoveTo(FieldLocation.BALL2_RED.getCoordinates().plus(new Translation2d(0.15, 0.30)), Rotation2d.fromDegrees(90))
				.UNSAFE_UNTIL((isAtTarget) -> isAtTarget || robot.ballHandlingHardware.ballAtBottom.get()),
			WAIT(BALL2_PATIENCE).UNSAFE_UNTIL((hasWaited) -> hasWaited || robot.ballHandlingHardware.ballAtBottom.get()),
			DO(robot.aiming::resume),
			WAIT(250), // wait for shooter
			robot.ballHandling.doFeedTwoBalls(0.5),
			WAIT(300),

			// Ball 3
			robot.drivetrain.doTurnTo(Rotation2d.fromDegrees(-10)),
			robot.drivetrain
				.doMoveTo(FieldLocation.BALL3_RED.getCoordinates().plus(new Translation2d(0, 0.15)), Rotation2d.fromDegrees(-10))
				.UNSAFE_UNTIL((isAtTarget) -> isAtTarget || robot.ballHandlingHardware.ballAtBottom.get()),
			WAIT(BALL3_PATIENCE).UNSAFE_UNTIL((hasWaited) -> hasWaited || robot.ballHandlingHardware.ballAtBottom.get()),
			WAIT(250), // wait for shooter
			robot.ballHandling.doFeedOneBall(),

			// Ball terminal and ball human player
			DO(robot.aiming::pauseOdometry),
			robot.drivetrain
				.doMoveTo(FieldLocation.BALL_TERMINAL_RED.getCoordinates().plus(new Translation2d(0.2, 0.05)), Rotation2d.fromDegrees(35))
				.UNSAFE_UNTIL((isAtTarget) -> isAtTarget || robot.ballHandlingHardware.ballAtBottom.get()),
			WAIT(BALL_TERMINAL_PATIENCE).UNSAFE_UNTIL((hasWaited) -> hasWaited || robot.ballHandlingHardware.ballAtBottom.get()),
			robot.drivetrain
				.doMoveTo(FieldLocation.BALL_TERMINAL_RED.getCoordinates().plus(new Translation2d(0.55, 0.85)), Rotation2d.fromDegrees(45)),
			WAIT(BALL_HUMAN_PLAYER_PATIENCE).UNSAFE_UNTIL((hasWaited) -> hasWaited || robot.ballHandling.isFull()),
			robot.drivetrain
				.doMoveTo(FieldLocation.BALL_TERMINAL_RED.getCoordinates().plus(new Translation2d(3.7, 0.5)), Rotation2d.fromDegrees(45)),
			DO(robot.aiming::resume),
			WAIT(500),
			robot.ballHandling.doFeedTwoBalls(0.8),

			DO(() -> System.out.println("Finished auto"))
		);
    }),
	SWEEP_LEFT(
		new Pose2d(
			FieldLocation.LEFT_SWEEP_STARTING_POSITION.getCoordinates(),
			Rotation2d.fromDegrees(27)
		),(robot) -> {
		final int SWEEP_DELAY = 0;

		return SEQUENCE(
			DO(() -> robot.shooter.shootWhileMoving(false)),
			DO(robot.aiming::pauseOdometry),
			DO(() -> robot.drivetrain.setPosition(FieldLocation.LEFT_SWEEP_STARTING_POSITION.getCoordinates(), Rotation2d.fromDegrees(27))),

			WAIT(SWEEP_DELAY),

			robot.drivetrain
				.doMoveTo(FieldLocation.BALL3_RED.getCoordinates().plus(new Translation2d(0.5, 0)), Rotation2d.fromDegrees(60))
				.UNSAFE_UNTIL((isAtTarget) -> isAtTarget || robot.ballHandlingHardware.ballAtBottom.get()),

			robot.drivetrain.doMoveTo(
				FieldLocation.RIGHT_WALL.getRelativeCoordinates(robot), new Translation2d(0, 0.3), 
				Rotation2d.fromDegrees(90),
				1
			),

			WAIT(100),
			robot.ballHandling.doFeedOneBall(),

			robot.drivetrain.doMoveTo(FieldLocation.RIGHT_WALL.getRelativeCoordinates(robot), new Translation2d(0, 0.5), 
				Rotation2d.fromDegrees(180),
				1
			),
			robot.drivetrain.doMoveRight(1.5, 0.8),
			
			DO(robot.aiming::resume),
			WAIT(100),
			DO(() -> robot.shooter.shootWhileMoving(true)),
			DO(robot.aiming::pauseOdometry),

			DO(() -> robot.drivetrain.setYPosition(FieldLocation.RIGHT_WALL.y)),

			robot.drivetrain.doMoveTo(new Translation2d(1, FieldLocation.RIGHT_WALL.y),
				Rotation2d.fromDegrees(180),
				1.4
			)
		);
	}),
	SWEEP_RIGHT(
		new Pose2d(
			FieldLocation.RIGHT_SWEEP_STARTING_POSITION.getCoordinates(),
			Rotation2d.fromDegrees(90)
		), (robot) -> {
		final int BALL1_PATIENCE = 1000; // the first blue ball
		final int SWEEP_DELAY = 0;

		return SEQUENCE(
			DO(() -> robot.drivetrain.setPosition(FieldLocation.RIGHT_SWEEP_STARTING_POSITION.getCoordinates(), Rotation2d.fromDegrees(90))),

			DO(() -> robot.shooter.shootStationary(true)),
			robot.drivetrain
				.doMoveTo(new Translation2d(0.6, -3.25), Rotation2d.fromDegrees(90), 0.5)
				.UNSAFE_UNTIL((isAtTarget) -> isAtTarget || robot.ballHandlingHardware.ballAtBottom.get()),
			WAIT(BALL1_PATIENCE).UNSAFE_UNTIL((hasWaited) -> hasWaited || robot.ballHandlingHardware.ballAtBottom.get()),
			WAIT(SWEEP_DELAY),

			robot.drivetrain.doMoveTo(new Translation2d(0.05, -2.8), Rotation2d.fromDegrees(180), 0.5),

			SWEEP_DOWN(robot)
		);
	}),
	ONE_BALL_RIGHT(
		new Pose2d(
			FieldLocation.RIGHT_SWEEP_STARTING_POSITION.getCoordinates(),
			Rotation2d.fromDegrees(90)
		), (robot) -> {

		return SEQUENCE(
			DO(() -> robot.drivetrain.setPosition(FieldLocation.RIGHT_SWEEP_STARTING_POSITION.getCoordinates(), Rotation2d.fromDegrees(90))),

			DO(() -> robot.shooter.shootStationary(true)),
			WAIT(2000),
			robot.drivetrain
				.doMoveTo(new Translation2d(0.6, -3.25), Rotation2d.fromDegrees(90), 0.5)
				.UNSAFE_UNTIL((isAtTarget) -> isAtTarget || robot.ballHandlingHardware.ballAtBottom.get())
		);
	}),
	TEST_FEED(
		new Pose2d(
			FieldLocation.RIGHT_SWEEP_STARTING_POSITION.getCoordinates(),
			Rotation2d.fromDegrees(90)
		), (robot) -> {

		return SEQUENCE(
			DO(() -> robot.drivetrain.setPosition(FieldLocation.RIGHT_SWEEP_STARTING_POSITION.getCoordinates(), Rotation2d.fromDegrees(90))),

			DO(() -> robot.shooter.shootStationary(false)),
			WAIT(2000),
			robot.ballHandling.doFeedTwoBalls(2)
		);
	});

	private static Actionable SWEEP_UP(Robot robot) {
		final double X_TARGET = 0;

		final double SPEED = 0.75;

		return SEQUENCE(
			DO(() -> robot.shooter.shootWhileMoving(true)),

			robot.drivetrain.doMoveTo(
				FieldLocation.RIGHT_WALL.getRelativeCoordinates(robot), new Translation2d(0, 0.5), 
				Rotation2d.fromDegrees(180),
				SPEED
			),
			DO(robot.aiming::pauseOdometry),
			robot.drivetrain.doMoveRight(2, 0.5),

			REPEAT(SEQUENCE(
				DO(() -> robot.drivetrain.setYPosition(FieldLocation.RIGHT_WALL.y)),
				DO(() -> robot.drivetrain.setHeading(new Rotation2d(180))),
				robot.drivetrain.doSweepToNextBallUp(
					FieldLocation.RIGHT_WALL.getRelativeCoordinates(X_TARGET), new Translation2d(0, 0), 
					Rotation2d.fromDegrees(180), 
					SPEED
				),
				WAIT(200)
			))
		);
	}

	private static Actionable SWEEP_DOWN(Robot robot) {
		final double X_TARGET = -6;

		final double SPEED = 0.75;

		return SEQUENCE(
			DO(() -> robot.shooter.shootStationary(true)),

			robot.drivetrain.doMoveTo(
				FieldLocation.RIGHT_WALL.getRelativeCoordinates(robot), new Translation2d(0, 0.15), 
				Rotation2d.fromDegrees(0), 
				SPEED
			),
			DO(robot.aiming::pauseOdometry),
			robot.drivetrain.doMoveRight(0.5, 0.5),

			REPEAT(SEQUENCE(
				DO(() -> robot.drivetrain.setYPosition(FieldLocation.RIGHT_WALL.y)),
				DO(() -> robot.drivetrain.setHeading(new Rotation2d(0))),
				robot.drivetrain.doSweepToNextBallDown(
					FieldLocation.RIGHT_WALL.getRelativeCoordinates(X_TARGET), new Translation2d(0, 0), 
					Rotation2d.fromDegrees(0), 
					SPEED
				),
				WAIT(200)
			))
		);
	}

	private enum FieldLocation {
		RIGHT_WALL(-3.84, true),
		RIGHT_SWEEP_STARTING_POSITION(0, -2.27),
		LEFT_SWEEP_STARTING_POSITION(-2, 0),
		RIGHT_SIDE_STARTING_POSITION(-0.65, -2.27),
		LEFT_SIDE_STARTING_POSITION(-1.67, 1.5),
		BALL1_BLUE(0.858, -3.790),
		BALL2_RED(-0.658, -3.830),
		BALL3_RED(-3.174, -2.243),
		BALL4_BLUE(-3.790, -0.858),
		BALL5_RED(-3.287, 2.074),
		BALL6_BLUE(-2.243, 3.174),
		BALL_TERMINAL_RED(-7.165, -2.990);

		private double x;
		private double y;

		private boolean xIsRelative;
		private boolean yIsRelative;

		FieldLocation(double x, double y) {
			this.x = x;
			this.y = y;
		}

		FieldLocation(double d, boolean xIsRelative) { // is xIsRelative is fase in this constructor then yIsRelative is true
			this.xIsRelative = xIsRelative;
			this.yIsRelative = !xIsRelative;

			if(this.yIsRelative) this.x = d;
			if(this.xIsRelative) this.y = d;
		}

		public Translation2d getCoordinates() {
			return new Translation2d(x, y);
		}

		public Gettable<Translation2d> getRelativeCoordinates(Robot robot) {
			return () -> new Translation2d(xIsRelative ? robot.drivetrain.getPosition().getX() : x, yIsRelative ? robot.drivetrain.getPosition().getY() : y);
		}

		public Gettable<Translation2d> getRelativeCoordinates(double target) {
			return () -> new Translation2d(xIsRelative ? target : x, yIsRelative ? target : y);
		}
	}

	public final Pose2d startPosition;
	private final Function<Robot, Actionable> actionableGenerator;

	private Auto(Pose2d startPosition, Function<Robot, Actionable> actionableGenerator) {
		this.startPosition = startPosition;
		this.actionableGenerator = actionableGenerator;
	}

	@Override
	public Actionable apply(Robot robot) {
		return actionableGenerator.apply(robot);
	}

	public static class AppliedAuto {
		public final Pose2d startingPosition;
		public final Actionable actionable;
		
		public AppliedAuto(Pose2d startingPosition, Actionable actionable) {
			this.startingPosition = startingPosition;
			this.actionable = actionable;
		}
	}
}