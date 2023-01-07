package redbacks.robot;

import java.io.File;

import arachne.lib.ArachneRobot;
import arachne.lib.dashboard.Dashboard;
import arachne.lib.dashboard.DefaultDashboard;
import arachne.lib.game.GameState;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import redbacks.robot.subsystems.drivetrain.Drivetrain;
import redbacks.robot.subsystems.drivetrain.DrivetrainBindings;
import redbacks.robot.subsystems.drivetrain.DrivetrainHardware;
import redbacks.robot.subsystems.climber.Climber;
import redbacks.robot.subsystems.climber.ClimberBindings;
import redbacks.robot.subsystems.climber.ClimberHardware;
import redbacks.robot.subsystems.intersystem.AimingToBallHandlingBindings;
import redbacks.robot.subsystems.intersystem.AimingToDrivetrainBindings;
import redbacks.robot.subsystems.intersystem.AimingToShooterBindings;
import redbacks.robot.subsystems.intersystem.AimingToTurretBindings;
import redbacks.robot.subsystems.intersystem.BallHandlingToDrivetrainBindings;
import redbacks.robot.subsystems.aiming.Aiming;
import redbacks.robot.subsystems.aiming.AimingBindings;
import redbacks.robot.subsystems.aiming.AimingHardware;
import redbacks.robot.subsystems.aiming.AimingRings;
import redbacks.robot.subsystems.intersystem.BallHandlingToShooterBindings;
import redbacks.robot.subsystems.intersystem.ClimberToRobotBindings;
import redbacks.robot.subsystems.intersystem.DrivetrainToShooterBindings;
import redbacks.robot.subsystems.ballhandling.BallHandling;
import redbacks.robot.subsystems.ballhandling.BallHandlingBindings;
import redbacks.robot.subsystems.ballhandling.BallHandlingHardware;
import redbacks.robot.subsystems.shooter.Shooter;
import redbacks.robot.subsystems.shooter.ShooterBindings;
import redbacks.robot.subsystems.shooter.ShooterHardware;
import redbacks.robot.subsystems.turret.Turret;
import redbacks.robot.subsystems.turret.TurretBindings;
import redbacks.robot.subsystems.turret.TurretHardware;

public class Robot extends ArachneRobot {
	public static void main(String[] args) {
		startRobot(Robot::new);
	}
	
	public final Controllers controllers = new Controllers();

	public final Drivetrain drivetrain = new Drivetrain();
	public final DrivetrainHardware drivetrainHardware = new DrivetrainHardware();
	
	public final Shooter shooter = new Shooter();
	public final ShooterHardware shooterHardware = new ShooterHardware();

	public final BallHandling ballHandling = new BallHandling(this);
	public final BallHandlingHardware ballHandlingHardware = new BallHandlingHardware();

	public final Aiming aiming = new Aiming();
	public final AimingHardware aimingHardware = new AimingHardware();

	public final Climber climber = new Climber();
	public final ClimberHardware climberHardware = new ClimberHardware();

	public final Turret turret = new Turret();
	public final TurretHardware turretHardware = new TurretHardware();

	public AutoManagerModified autos;

	// Reinit control variables
	private double bootUpTime = Double.NaN;
	private int reinitCount = 0;

	private boolean firstInit = true;
	private boolean hasRunAuto = false;

	public Robot() {
		super(Constants.LOOP_PERIOD);
	}

	@Override
	protected void initialize() {
		Dashboard.setImplementation(new DefaultDashboard());
		Dashboard.getInstance().putNumber("Sweep Delay Seconds", 0);

		drivetrainHardware.initialize();
		DrivetrainBindings.bind(drivetrain, drivetrainHardware, controllers);

		climberHardware.initialize();
		ClimberBindings.bind(climber, climberHardware, drivetrainHardware.pitchDegrees, controllers);

		shooterHardware.initialize();
		ShooterBindings.bind(shooter, shooterHardware, controllers);

		ballHandlingHardware.initialize();
		BallHandlingBindings.bind(ballHandling, ballHandlingHardware, controllers);

		turretHardware.initialize();
		TurretBindings.bind(turret, turretHardware);

		AimingRings.initialize(new File(Filesystem.getDeployDirectory(), "AimingMap.csv"));

		aimingHardware.initialize();
		AimingBindings.bind(aiming, aimingHardware, controllers);

		AimingToBallHandlingBindings.bind(aiming, ballHandling);
		AimingToDrivetrainBindings.bind(aiming, drivetrain);
		AimingToShooterBindings.bind(aiming, shooter);
		AimingToTurretBindings.bind(aiming, turret);
		BallHandlingToShooterBindings.bind(ballHandling, shooter);
		ClimberToRobotBindings.bind(climber, this);
		DrivetrainToShooterBindings.bind(drivetrain, shooter);
		BallHandlingToDrivetrainBindings.bind(ballHandling, drivetrain);

		drivetrain.initialize();
		climber.initialize();
		shooter.initialize();
		ballHandling.initialize();
		aiming.initialize();
		turret.initialize();

		autos = new AutoManagerModified(this, Auto.DO_NOTHING, Auto.values());
	}

	@Override
	protected void onStateChange(GameState oldState, GameState newState) {
		drivetrain.onGameStateChange(newState);
		aiming.resume();

		if(newState == GameState.TELEOP) {
			shooter.shootStationary(false);
			ballHandling.intake();
		}
		else if(newState == GameState.AUTO) {
			drivetrainHardware.navx.reset();
			autos.updateStartingPositionFromSelected(drivetrain);
			autos.startAuto();
			hasRunAuto = true;
		}
		else autos.stopAuto();
	}

	@Override
	protected void execute(GameState state) {
		autos.run();

		var currentPose = drivetrain.getPosition();
        Dashboard.getInstance().putNumber("Forward", currentPose.getTranslation().getX());
        Dashboard.getInstance().putNumber("Left", currentPose.getTranslation().getY());
        Dashboard.getInstance().putNumber("Rotation", currentPose.getRotation().getDegrees());

		Dashboard.getInstance().putString("Ball colour", ballHandlingHardware.currentColour.get().toString());

		if(state != GameState.DISABLED || firstInit) {
			try {
				aiming.run();
			}
			catch(Exception e) {
				System.err.println("Error in aiming calculations! Skipping to next subsystem...");
				e.printStackTrace();
			}

			drivetrain.run();
			climber.run();
			shooter.run();
			ballHandling.run();
			turret.run();

			if(firstInit) {
				firstInit = false;
				bootUpTime = Timer.getFPGATimestamp();
			}
		}
		else {
			if(reinitCount < Constants.REINIT_ATTEMPT_LIMIT && Timer.getFPGATimestamp() - bootUpTime > reinitCount * Constants.REINIT_DELAY) {
				drivetrainHardware.reinit();
				reinitCount++;
				System.out.println("Re-init #" + reinitCount + " completed");
			}
		}
	}
}
