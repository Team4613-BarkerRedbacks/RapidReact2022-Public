package redbacks.robot;

import edu.wpi.first.wpilibj.XboxController;

public class Controllers {
	private static final double TRIGGER_DISTANCE_FOR_PRESS = 0.1;

	public final XboxController 
		driver = new XboxController(0),
		operator = new XboxController(1);

	public static final boolean isTriggerPressed(double triggerValue) {
		return triggerValue >= TRIGGER_DISTANCE_FOR_PRESS;
	}
}
