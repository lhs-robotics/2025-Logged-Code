package frc.robot.subsystems.Algae;

import frc.robot.subsystems.coral.arm.ArmIO.ArmIOInputs;

public interface AlgaeIO {
    public default void updateInputs(ArmIOInputs inputs) {
    }

    public default void setAlgaeSystemAngleDegrees(double angle) {
    }
    public default void setAlgaeMotorSpeed(double speed) {}
}
