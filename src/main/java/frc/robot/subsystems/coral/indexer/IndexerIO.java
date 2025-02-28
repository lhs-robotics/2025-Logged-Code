package frc.robot.subsystems.coral.indexer;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface IndexerIO {
    @AutoLog
    public static class IndexerIOInputs {
        public boolean connected = false;
        public double indexMotorAppliedOutput = 0;
        public boolean autoLoadTriggerTripped = false;
    }

    public default void enableIndexer() {
    }

    public default void disableIndexer() {
    }

    public default Trigger getAutoLoadTrigger() {
        return null;
    }

    public default void updateInputs(IndexerIOInputs inputs) {}

}
