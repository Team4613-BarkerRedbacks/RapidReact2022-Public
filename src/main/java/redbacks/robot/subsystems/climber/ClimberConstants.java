package redbacks.robot.subsystems.climber;

public class ClimberConstants {
    private static final double MAX_TRAVEL_DISTANCE_TICKS = 235000;

    static final double 
        HOOKS_AT_TOP_HEIGHT_METRES = 1.589,
        HOOKS_AT_BOTTOM_HEIGHT_METRES = 0.803,
        HOOKS_AT_TARGET_POSITION_TOLERANCE_METRES = 0.02,
        HOOKS_COAST_TIMEOUT_SECONDS = 0.5,
        HOOKS_COAST_TRANSFER_HEIGHT = 0.855,
        RATCHET_UNLOCK_TIME_SECONDS = 1,
        RETRACTION_ON_FLOOR_POWER = -0.7,
        RETRACTION_ON_BAR_POWER = -0.7,
        CLIMBING_TARGET_LEAN_PITCH = 50,
        CLIMBING_TARGET_RETRACT_PITCH = 42,
        TICKS_TO_METRES = (HOOKS_AT_TOP_HEIGHT_METRES - HOOKS_AT_BOTTOM_HEIGHT_METRES) / MAX_TRAVEL_DISTANCE_TICKS,

        HOOKS_SEMI_RAISED_TARGET_POSITION = 1.1,

        HAND_OVER_TARGET_POWER = 0.1,
        HAND_OVER_TARGET_POSITION = HOOKS_AT_BOTTOM_HEIGHT_METRES + 0.1,
        TARGET_EXTENSION_TO_BAR = 1.2,
        FINAL_TARGET_POSITION = 1.3,

        KP = 0.025,
        KI = 0,
        KD = 0,
        MAX_PID_OUTPUT = 0.65;
}