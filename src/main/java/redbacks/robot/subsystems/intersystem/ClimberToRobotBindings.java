package redbacks.robot.subsystems.intersystem;

import arachne.lib.scheduler.ScheduledBooleanSource;
import redbacks.robot.Robot;
import redbacks.robot.subsystems.climber.Climber;

public class ClimberToRobotBindings {
    public static void bind(Climber climber, Robot robot) {
        climber
        .addBinding(new ScheduledBooleanSource(false, () -> climber.getState() != Climber.SystemState.PRE_CLIMB))
        .attach((isClimbing) -> {
            if(isClimbing) {
                robot.ballHandling.stop();
                robot.aiming.stowForClimb();
            }
            else {
                robot.ballHandling.intake();
                robot.aiming.resume();
            }
        });
    }
}
