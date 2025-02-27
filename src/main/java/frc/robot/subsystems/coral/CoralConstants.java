package frc.robot.subsystems.coral;

public class CoralConstants {
    public final static int canRangeID = 30;
    public final static boolean autoLoadEnabled = true;
    public static final double autoLoadMaxDistance = 2;
    public enum CoralState {
        kStow,
        kPreLoad, // Preps coral to load
        kTransport, // Moving coral to specific location
        kL1,
        kL2,
        kL3,
        kL4
    }
}
