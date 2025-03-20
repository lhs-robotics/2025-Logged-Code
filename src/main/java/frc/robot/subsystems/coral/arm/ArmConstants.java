package frc.robot.subsystems.coral.arm;

import edu.wpi.first.math.util.Units;

public class ArmConstants {
  public static final int gearboxSparkID = 24;
  public static final double gearboxRatio =  (72/42)/12;

  public static final double coralReleaseTimeSecs = 2;

  public static final double loadAngle = 8.571456909179688;
  public static final double homeAngle = 0;
  public static final double level1Angle = 2.5476183891296387;
  public static final double level2Angle = 0;
  public static final double level3Angle = 0;
  public static final double level4Angle = 0;

  public static final double manualVelocity = 0;

  public static final double positionP = 1;
  public static final double positionD = 0.0;

  public static final double velocityP = 0;
  public static final double velocityI = 0;
  public static final double velocityD = 0;

  public static final double maxVelocity = 0;
  public static final double maxAcceleration = 0;

  public static final int gearBoxCurrentLimit = 50;
  public static final boolean gearBoxInveted = true;

  public static final double coralIntakeSpeed = -.75;
  public static final double coralReleaseSpeed = .75;
  public static final int endAffectorSparkID = 25;
  public static final int endAffectorCurrentLimit = 40;
  public static final boolean endAffectorInverted = false;
  public static final double minElevatorRotateHeight = 7;
public static final double kG = 0;
public static final double kA = 0;
public static double kS;
}
