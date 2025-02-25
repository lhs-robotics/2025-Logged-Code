package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.XboxController;

public class DriverFeedback extends SubsystemBase {

    private final CommandXboxController operatorController;
    private int blininPin = 9;

    public DriverFeedback(){

        operatorController = new CommandXboxController(1);


    }
    

}
