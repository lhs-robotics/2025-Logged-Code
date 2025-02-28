package frc.robot.subsystems.coral.arm;

import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
  @AutoLog
  public static class ArmIOInputs {
    public boolean connected = false;
    public double height = 0;
    public double velocityRPM = 0.0;
    public boolean atTarget = false;
  }

  public default void updateInputs(ArmIOInputs inputs) {
  }

  public default void setArmAngleDegrees(double heightInches) {
  }

  public default void runCharacterization(double output) {
  }

  public default void setVelocity(double velocity) {
  }

  public default void setPositionPID(double kP, double kI, double kD) {
  }

  public default void setMaxVelocityAcceleration(double velocity, double acceleration) {
  }

  public default void enableEndAffectorBrake() {
  }

  public default void disableEndAffectorBrake() {
  }
}
