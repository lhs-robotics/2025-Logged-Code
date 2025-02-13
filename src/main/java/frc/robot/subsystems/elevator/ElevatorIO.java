package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface ElevatorIO {
    @AutoLog
    public static class ElevatorIOInputs {
        public boolean connected = false;
        public double height = 0;
        public double velocityRPM = 0.0;
    }

    public default void updateInputs(ElevatorIOInputs inputs) {}

    public default void setElevatorHeightInches(double heightInches) {}

    public default void runCharacterization(double output) {}

    public default void setVelocity(double velocity) {}

}
