package redbacks.robot;

import arachne.lib.scheduler.Schedulable;
import arachne.lib.sequences.ActionConductor;
import arachne.lib.sequences.Actionable;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import redbacks.robot.Auto.AppliedAuto;
import redbacks.robot.subsystems.drivetrain.Drivetrain;

public class AutoManagerModified implements Schedulable
{
	protected static final String DASHBOARD_TAB = "Autonomous";
	protected static final String DEFAULT_SELECTION_KEY = "Auto selection";
	
	protected final ActionConductor conductor;
	protected final SendableChooser<AppliedAuto> autoChooser;
	
	public AutoManagerModified(Robot robot, Auto defaultAuto, Auto[] autos) {
		this(robot, defaultAuto, autos, DEFAULT_SELECTION_KEY);
	}
	
	public AutoManagerModified(Robot robot, Auto defaultAuto, Auto[] autos, String selectionKey) {		
		this.conductor = new ActionConductor();
		
		this.autoChooser = new SendableChooser<AppliedAuto>();
		this.autoChooser.setDefaultOption(defaultAuto.toString(), new AppliedAuto(defaultAuto.startPosition, defaultAuto.apply(robot)));
		
		for(Auto auto : autos) {
			if(auto != defaultAuto) autoChooser.addOption(auto.toString(), new AppliedAuto(auto.startPosition, auto.apply(robot)));
		}

		Shuffleboard.getTab(DASHBOARD_TAB).add(selectionKey, autoChooser);
	}
	
	public void startAuto() {
		var auto = autoChooser.getSelected();
		startAuto(auto == null ? Actionable.DO_NOTHING() : auto.actionable);
	}

    public void updateStartingPositionFromSelected(Drivetrain drivetrain) {
        var auto = autoChooser.getSelected();
        if(auto != null) drivetrain.setPosition(auto.startingPosition.getTranslation(), auto.startingPosition.getRotation());
    }
	
	public void startAuto(Actionable auto) {
		stopAuto();
		conductor.add(auto);
	}
	
	public void stopAuto() {
		conductor.interrupt();
	}
	
	public boolean isRunning() {
		return conductor.hasActions();
	}

	@Override
	public void run() {
		conductor.run();
	}
}
