package frc.robot.subsystems.feedback;

import java.util.Map;

import frc.robot.subsystems.StateManager.RobotState;

public class FeedbackConstants {

    public static enum FeedbackType {
        loaded,
        aligned,
        released
    }

    public static final int blinkinPort = 0;

    // All lengths in seconds
    public static final double modeChangeVibrateLength = 0.5;
    public static final double modeChangeTimeBetweenVibrations = 0.5;

    public static final double feedbackVibrationLength = 1;
    public static final double feedbackLedLength = 5;

    public static final double vibrationIntensity = 1;

    // TODO: EXAMPLE VALUES - REPLACE W/ ACTUAL
    public static final Map<RobotState, Double> ledModeTable = Map.of(RobotState.coral, .99,
            RobotState.algae, .98);

    public static final Map<FeedbackType, Double> ledFeedbackTable = Map.of(FeedbackType.loaded, .99,
            FeedbackType.released,
            .98);

}
