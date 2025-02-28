package frc.robot.subsystems.algae;

import org.littletonrobotics.junction.AutoLog;

public interface AlgaeIO {
    @AutoLog
    public static class AlgaeIOInputs {
        public double algaeMotorAppliedOutput = 0;
        public boolean algaeMotorConnected = false;

        public boolean autoIntaking = false;
    }

    public default void updateInputs(AlgaeIOInputs inputs) {
    }

    public default void setAlgaeSystemAngleDegrees(double angle) {
    }

    public default void setAlgaeMotorSpeed(double speed) {
    }

    public default void beginAutoIntake() {
    }
}
