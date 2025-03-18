package frc.robot.subsystems.climb;

import org.littletonrobotics.junction.AutoLog;

public interface ClimbIO {
    @AutoLog
    public static class ClimbIOInputs {
        public boolean connected = false;
        public double positionDegrees = 0;
        public double velocityRPM = 0.0;
        public boolean atTarget = false;
    }
    public default void stopClimb() {
    }
    public default void runClimb(boolean movingIn){
    }
    public default void setPositionDegrees(double angle) {
    }

    public default void updateInputs(ClimbIOInputs inputs) {
    }

}
