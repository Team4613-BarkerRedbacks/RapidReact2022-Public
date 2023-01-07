package redbacks.robot.arachne.lib.io.sensors;

import arachne.lib.io.GettableBoolean;
import edu.wpi.first.wpilibj.Timer;

public class StickyGettableBoolean implements GettableBoolean {
    protected final GettableBoolean gettable;
    protected final double stickDurationSeconds;

    protected double trueUntilTime = Double.NaN;

    public StickyGettableBoolean(GettableBoolean gettable, double stickDurationSeconds) {
        this.gettable = gettable;
        this.stickDurationSeconds = stickDurationSeconds;
    }

    @Override
    public boolean get() {
        var gettableValue = gettable.get();

        if(gettableValue) {
            trueUntilTime = Timer.getFPGATimestamp() + stickDurationSeconds;
            return true;
        }
        else {
            return Timer.getFPGATimestamp() < trueUntilTime;
        }
    }
}
