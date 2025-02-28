package frc.robot.subsystems.coral.arm;

public class ArmConstants {
  public static final int gearboxSparkID = 25;
  public static final double gearboxRatio = (72 / 42) * 12;

  public static final double coralReleaseTimeSecs = 2;

  public static final double loadAngle = 0;
  public static final double homeAngle = 0;
  public static final double level1Angle = 0;
  public static final double level2Angle = 0;
  public static final double level3Angle = 0;
  public static final double level4Angle = 0;

  public static final double manualVelocity = 0;

  public static final double positionP = .4;
  public static final double positionD = 0;

  public static final double velocityP = 0;
  public static final double velocityI = 0;
  public static final double velocityD = 0;

  public static final double maxVelocity = 0;
  public static final double maxAcceleration = 0;

  public static final int gearBoxCurrentLimit = 50;
  public static final boolean gearBoxInveted = true;

  public static final double coralIntakeSpeed = -.75;
  public static final double coralReleaseSpeed = .75;
  public static final int endAffectorSparkID = 0;
  public static final int endAffectorCurrentLimit = 40;
  public static final boolean endAffectorInverted = false;
}
