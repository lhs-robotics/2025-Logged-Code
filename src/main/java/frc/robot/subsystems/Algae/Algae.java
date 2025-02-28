package frc.robot.subsystems.algae;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Algae extends SubsystemBase {
    private final AlgaeIO algaeIO;

    public Algae(AlgaeIO algaeIO) {
        this.algaeIO = algaeIO;
    }

    public void disable() {
        algaeIO.setAlgaeSystemAngleDegrees(AlgaeConstants.homeAngle);
    }

    public void enable() {
        algaeIO.setAlgaeSystemAngleDegrees(AlgaeConstants.intakeAngle);

    }

    public void manualIntakeAlgaeBegin() {
        Logger.recordOutput("Algae/intakeSpeed", AlgaeConstants.intakeSpeed);
        algaeIO.setAlgaeMotorSpeed(AlgaeConstants.intakeSpeed);
    }
    public void manualIntakeAlgaeEnd() {
        Logger.recordOutput("Algae/intakeSpeed", 0);
        algaeIO.setAlgaeMotorSpeed(0);
    }
    public void releaseAlgae() {
        algaeIO.setAlgaeMotorSpeed(AlgaeConstants.releaseSpeed);
    }
    
}
