package frc.robot.subsystems.coral.elevator;

public class ElevatorConstants {
  public static final int motor1ID = 21;
  public static final int motor2ID = 22;

  public static final int L1Button = 1;
  public static final int L2Button = 2;
  public static final int L3Button = 3;
  public static final int L4Button = 4;
  public static final int HomeButton = 5;

  public static final int ManualDownButton = 7;
  public static final int ManualUpButton = 6;

  public static final int elevatorMotorSmartCurrentLimit = 40;

  public static final boolean motor1Inveted = true;
  public static final boolean elevatorMotor2Inverted = false;

  public static final double positionP = 1;
  public static final double positionI = 0;
  public static final double positionD = 0;

  public static final double positionConversionFactor = 1; //0.7493

  public static final double maxVelocity = 500;
  public static final double maxAcceleration = 750;
  public static final double allowedError = 0.1;

  public static final double preLoadHeight = 16.35712432861328;
  public static final double homeHeight = 0;
  public static final double loadedHeight = 9.714326858520508;
  public static final double level1Height = 7.928591251373291;
  public static final double level2Height = 0;
  public static final double level3Height = 0;
  public static final double level4Height = 0;

  public static final double manualVelocity = 0;

  public static final double kS = 0;
  public static final double kG = 0;
  public static final double kV = 0;
  public static final double kA = 0;
  
  public static final double velocityP = 0;
  public static final double velocityI = 0;
  public static final double velocityD = 0;
}
