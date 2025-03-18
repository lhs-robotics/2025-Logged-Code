package frc.robot.subsystems.coral.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
  @AutoLog
  public static class ElevatorIOInputs {
    // *Is elevator motor connected? */
    public boolean connected = false;
    // *Elevator height from elevator 0 (not robot base) - given in inches */
    public double height = 0;
    // *Elevator motor velocity in RPM */
    public double velocityRPM = 0.0;
    // *Boolean if elevator is at height setpoint within margin of error */
    public boolean atTarget = false;
    // TODO: ADD CARRIAGE VELOCITY
    public double appliedVoltage = 0.0;
  }
  public default void manualElevatorStart(boolean up) {

  }
  public default void manualElevatorStop() {}
  public default void updateInputs(ElevatorIOInputs inputs) {}

  public default void enableBreak() {}

  public default void disableBreak(){}

  public default void setElevatorHeightInches(double heightInches) {}

  public default void runCharacterization(double output) {}

  public default void setVelocity(double velocity) {}

  public default void setPositionPID(double kP, double kI, double kD) {}

  public default void setMaxVelocityAcceleration(double velocity, double acceleration) {}
  ;
}
