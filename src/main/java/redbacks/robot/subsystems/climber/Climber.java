package redbacks.robot.subsystems.climber;

import arachne.lib.dashboard.Dashboard;
import arachne.lib.io.GettableBoolean;
import arachne.lib.io.GettableDouble;
import arachne.lib.io.sensors.GettableBooleanInput;
import arachne.lib.io.sensors.GettableDoubleInput;
import arachne.lib.io.sensors.Input;
import arachne.lib.io.sensors.SettableDoubleInput;
import arachne.lib.io.sensors.SettableDoubleSensor;
import arachne.lib.pipeline.BooleanPipe;
import arachne.lib.pipeline.BooleanSource;
import arachne.lib.pipeline.DoublePipe;
import arachne.lib.pipeline.DoubleSource;
import arachne.lib.sequences.Actionable;
import arachne.lib.sequences.actions.Action;
import arachne.lib.states.State;
import arachne.lib.states.StatefulSubsystem;

import static redbacks.robot.subsystems.climber.ClimberConstants.*;
import static arachne.lib.sequences.Actionable.*;

public class Climber extends StatefulSubsystem<Climber.SystemState, Climber> {
    // Outputs
    private final BooleanPipe unlock, secondaryHooks, coastMode;
    private final DoublePipe mainHooksPower, mainHooksTargetPosition;

    // Sensors
    private final GettableBooleanInput hookAtBottom;
    private final GettableDoubleInput robotPitch;
    private final SettableDoubleInput position;

    // Control variables
    private boolean overridePitch = false;

    public enum SystemState implements State<SystemState, Climber> {
        MANUAL {
            @Override
            void acceptManualRatchetUnlockInput(Climber climber, boolean activate) {
                climber.unlock.accept(activate);
            }

            @Override
            void acceptManualMainHooksInput(Climber climber, double speed) {
                // If the climber is at the bottom and is trying to go down then it stops.
                climber.mainHooksPower.accept(climber.hookAtBottom.get() && speed < 0 ? 0 : speed);
            }

            @Override
            void acceptManualExtendsSecondaryHooksInput(Climber climber, boolean activate) {
                climber.secondaryHooks.accept(activate);
            }
        },
        PRE_CLIMB {
            @Override
            void raiseHooks(Climber climber) {
                climber.setState(HOOKS_SEMI_RAISED);
            }
        },
        HOOKS_SEMI_RAISED {
            @Override
            public Action createStateAction(Climber climber) {
                return SEQUENCE(
                    DO(() -> climber.unlock.accept(true)),
                    WAIT((long) (RATCHET_UNLOCK_TIME_SECONDS * 1000)),
                    DO(() -> climber.mainHooksTargetPosition.accept(HOOKS_SEMI_RAISED_TARGET_POSITION))
                ).asAction(null);
            }

            @Override
            void raiseHooks(Climber climber) {
                if(climber.currentStateAction.hasFinished()) climber.setState(HOOKS_FULLY_RAISED);
            }
        },
        HOOKS_FULLY_RAISED {
            @Override
            public Action createStateAction(Climber climber) {
                return SEQUENCE(
                    DO(() -> climber.mainHooksTargetPosition.accept(HOOKS_AT_TOP_HEIGHT_METRES)),
                    WAIT().UNSAFE_UNTIL(climber::areHooksAtTop)
                ).asAction(null);
            }

            @Override
            void climb(Climber climber) {
                if(climber.currentStateAction.hasFinished()) climber.setState(CLIMBING);
            }
        },
        CLIMBING {
            @Override
            public Action createStateAction(Climber climber) {
                return SEQUENCE(
                    DO(() -> climber.mainHooksPower.accept(RETRACTION_ON_FLOOR_POWER)),
                    WAIT().UNSAFE_UNTIL(climber.hookAtBottom),
                    moveToNextBar(climber),
                    WAIT().UNSAFE_UNTIL(() -> climber.robotPitch.get() < CLIMBING_TARGET_RETRACT_PITCH || climber.overridePitch),
                    DO(() -> climber.mainHooksPower.accept(RETRACTION_ON_BAR_POWER)),
                    WAIT().UNSAFE_UNTIL(climber.hookAtBottom),

                    moveToNextBar(climber),
                    WAIT(1500),
                    WAIT().UNSAFE_UNTIL(() -> climber.robotPitch.get() < CLIMBING_TARGET_RETRACT_PITCH || climber.overridePitch),
                    DO(() -> climber.mainHooksPower.accept(RETRACTION_ON_BAR_POWER)),
                    WAIT().UNSAFE_UNTIL(() -> climber.position.get() < FINAL_TARGET_POSITION),
                    DO(() -> climber.mainHooksPower.accept(0)),
                    DO(() -> climber.unlock.accept(false))
                ).asAction(null);
            }

            private Actionable moveToNextBar(Climber climber) {
                return SEQUENCE(
                    DO(() -> climber.mainHooksPower.accept(0)),
                    DO(() -> climber.coastMode.accept(true)),
                    WAIT((long) (HOOKS_COAST_TIMEOUT_SECONDS * 1000)).UNSAFE_UNTIL((hasWaited) -> hasWaited || climber.position.get() > HOOKS_COAST_TRANSFER_HEIGHT),
                    DO(() -> climber.mainHooksTargetPosition.accept(TARGET_EXTENSION_TO_BAR)),
                    WAIT().UNSAFE_UNTIL(() -> climber.position.get() > HAND_OVER_TARGET_POSITION),
                    DO(() -> climber.secondaryHooks.accept(true)),
                    DO(() -> climber.coastMode.accept(false)),
                    WAIT().UNSAFE_UNTIL(() -> climber.robotPitch.get() > CLIMBING_TARGET_LEAN_PITCH || climber.overridePitch),
                    DO(() -> climber.mainHooksTargetPosition.accept(HOOKS_AT_TOP_HEIGHT_METRES)),
                    WAIT().UNSAFE_UNTIL(climber::areHooksAtTop),
                    DO(() -> climber.secondaryHooks.accept(false))
                );
            }
        };

        void climb(Climber climber) {};
        void raiseHooks(Climber climber) {};
        void acceptManualRatchetUnlockInput(Climber climber, boolean activate) {}
        void acceptManualMainHooksInput(Climber climber, double speed) {}
        void acceptManualExtendsSecondaryHooksInput(Climber climber, boolean activate) {}
    }

    public Climber() {
        super(SystemState.PRE_CLIMB, SystemState.class);

        this.unlock = new BooleanPipe(false);
        this.secondaryHooks = new BooleanPipe(false);
        this.coastMode = new BooleanPipe(false);
        this.mainHooksPower = new DoublePipe(0);
        this.mainHooksTargetPosition = new DoublePipe(0);

        this.hookAtBottom = new GettableBooleanInput();
        this.robotPitch = new GettableDoubleInput();
        this.position = new SettableDoubleInput();
    }

    @Override
    protected Climber getSelf() {
        return this;
    }

    @Override
    public void run() {
        super.run();

        Dashboard.getInstance().putNumber("Climber position", position.get());
        Dashboard.getInstance().putBoolean("Climber fully retracted", hookAtBottom.get());
        Dashboard.getInstance().putNumber("Robot Pitch", robotPitch.get());
    }

    public void raiseHooks() {
        state.raiseHooks(this);
    }

    public void climb() {
        state.climb(this);
    }

    public void acceptManualRatchetUnlockInput(boolean activate) {
        state.acceptManualRatchetUnlockInput(this, activate);
    }

    public void acceptManualMainHooksInput(double speed) {
        state.acceptManualMainHooksInput(this, speed);
    }

    public void acceptManualExtendsSecondaryHooksInput(boolean activate) {
        state.acceptManualExtendsSecondaryHooksInput(this, activate);
    }

    public void overridePitch(boolean shouldOverride) {
        overridePitch = shouldOverride;
    }

    private boolean areHooksAtTop() {
        return position.get() > HOOKS_AT_TOP_HEIGHT_METRES - HOOKS_AT_TARGET_POSITION_TOLERANCE_METRES;
    }

    public BooleanSource getUnlockOutput() {
        return unlock;
    }

    public BooleanSource getSecondaryHooksOutput() {
        return secondaryHooks;
    }

    public DoubleSource getMainHooksPowerOutput() {
        return mainHooksPower;
    }

    public DoubleSource getMainHooksTargetPositionOutput() {
        return mainHooksTargetPosition;
    }

    public BooleanSource getCoastModeOutput() {
        return coastMode;
    }

    public Input<GettableBoolean> getHookAtBottomSensor() {
        return hookAtBottom;
    }

    public Input<GettableDouble> getRobotPitchSensor() {
        return robotPitch;
    }

    public Input<SettableDoubleSensor> getPositionSensor() {
        return position;
    }
}
