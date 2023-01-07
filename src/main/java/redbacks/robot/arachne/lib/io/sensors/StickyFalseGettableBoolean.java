package redbacks.robot.arachne.lib.io.sensors;

import arachne.lib.io.GettableBoolean;
import edu.wpi.first.wpilibj.Timer;

public class StickyFalseGettableBoolean implements GettableBoolean {
    protected final GettableBoolean gettable;
    protected final double stickDurationSeconds;

    protected double falseUntilTime = Double.NaN;

    public StickyFalseGettableBoolean(GettableBoolean gettable, double stickDurationSeconds) {
        this.gettable = gettable;
        this.stickDurationSeconds = stickDurationSeconds;
    }

    @Override
    public boolean get() {
        var gettableValue = gettable.get();

        if(!gettableValue) {
            falseUntilTime = Timer.getFPGATimestamp() + stickDurationSeconds;
            return false;
        }
        else {
            return Timer.getFPGATimestamp() > falseUntilTime;
        }
    }
}
