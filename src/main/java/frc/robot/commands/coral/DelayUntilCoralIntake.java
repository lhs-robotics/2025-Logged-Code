package frc.robot.commands.coral;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.coral.CoralSystem;

public class DelayUntilCoralIntake extends Command {
    private final Trigger autoLoadTrigger;

    public DelayUntilCoralIntake(CoralSystem coralSys) {
        this.autoLoadTrigger = coralSys.getAutoLoadTrigger();
    }
    @Override
    public boolean isFinished() {
        return autoLoadTrigger.getAsBoolean();
    }

    
}
