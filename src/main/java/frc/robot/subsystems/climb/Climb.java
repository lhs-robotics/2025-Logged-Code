package frc.robot.subsystems.climb;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Climb extends SubsystemBase {
    private final ClimbIO climbIO;
    private final ClimbIOInputsAutoLogged climbInputs = new ClimbIOInputsAutoLogged();

    public Climb(ClimbIO climbIO){
        this.climbIO = climbIO; 
    }

    public void setClimbOutBumper() {
        System.out.println("CLIMB");
        climbIO.setPositionDegrees(ClimbConstants.insideBumperAngle);
    }

    /**
     * Moves climb inside of bumpers - uses PID to either climb with high force or just pop in
     */
    public void setClimbInBumper() {
        System.out.println("CLIMB");

        climbIO.setPositionDegrees(ClimbConstants.outsideBumperAngle);

    }

    @Override
    public void periodic() {
        climbIO.updateInputs(climbInputs);
        Logger.processInputs("Climb", climbInputs);
    }

    public void disable() {
        setClimbInBumper();
    }

    public void enable() {
        setClimbOutBumper();
    }


}
