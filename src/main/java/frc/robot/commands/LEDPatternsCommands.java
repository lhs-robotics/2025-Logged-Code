package frc.robot.commands;

import frc.robot.subsystems.DriverFeedback;
import edu.wpi.first.wpilibj2.command.Command;

public class LEDPatternsCommands extends Command {
    
    private final DriverFeedback driverFeedback;
    private final double patternValue;

    public LEDPatternsCommands(DriverFeedback subsystem, double patternValue) {
        this.driverFeedback = subsystem;
        this.patternValue = patternValue;
    }

    @Override
    public void initialize() {
        driverFeedback.setLEDPattern(patternValue);
    }

    @Override 
    public boolean isFinished(){
        return true;
    }
}