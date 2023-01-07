package redbacks.robot;

public final class Constants {
    public static final String CANIVORE_BUS_NAME = "canivore";

	public static final int
        CAN_TIMEOUT = 0,
        CANCODER_TIMEOUT = 100,
        CANCODER_STATUS_FRAME_PERIOD = 250,
        REINIT_ATTEMPT_LIMIT = 5;

    public static final double
        LOOP_PERIOD = 0.01,
        REINIT_DELAY = 1;

    public static final double
        TALON_FX_VELOCITY_MAX_OUTPUT = 1023,
        TALON_FX_TICKS_PER_REVOLUTION = 2048,
        MAG_ENCODER_TICKS_PER_REVOLUTION = 4096;
}
