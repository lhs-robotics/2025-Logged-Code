package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.drive.Drive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.Spark;

public class DriverFeedback extends SubsystemBase {

    private final Spark blinkin;

    // private final CommandXboxController operatorController;

    private int blininPin = 9;

    public DriverFeedback(){
        blinkin = new Spark(0);

        // operatorController = new CommandXboxController(1);
    }

    public void setLEDPattern(double value){
        blinkin.set(value);
    }
    


}
