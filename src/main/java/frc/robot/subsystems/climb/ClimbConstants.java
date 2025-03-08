package frc.robot.subsystems.climb;

public class ClimbConstants {

    public static final int climbMotorID = 20;
    public static final double speed = 0;
    public static final boolean climbMotorInverted = true;
    public static final double insideBumperAngle = 0.0;
    public static final double outsideBumperAngle = 25.3;

    public static final double positionP = 0.5;
    public static final double positionI = 0;
    public static final double positionD = 0;

    public static final double positionConversionFactor = 6.25;
    public static final double velocityConversionFactor = 6.25;

    public static final double maxVelocity = 20; // Degrees per minute
    public static final double maxAcceleration = 30; // Degrees per minute per second
    public static final double allowedError = 1; // Degrees
}
