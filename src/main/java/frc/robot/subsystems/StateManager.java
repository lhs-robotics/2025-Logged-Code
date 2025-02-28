package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Algae.Algae;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.coral.CoralSystem;
import frc.robot.subsystems.feedback.DriverFeedback;

public class StateManager extends SubsystemBase {
    public enum RobotState {
        defense,
        coral,
        algae,
        climb
    }

    private RobotState currentState;

    private final CoralSystem coralSys;
    private final Climb climb;
    private final Algae algae;
    private final DriverFeedback driverFeedback;

    public StateManager(CoralSystem coralSys, Climb climb, Algae algae,
            DriverFeedback driverFeedback) {
        this.coralSys = coralSys;
        this.climb = climb;
        this.algae = algae;
        this.driverFeedback = driverFeedback;
    }

    public RobotState getCurrentState() {
        return currentState;
    }

    public void updateRobotState(RobotState newState) {
        Logger.recordOutput("StateMachine/currentState", newState);
        currentState = newState;
        // Disable current state
        switch (currentState) {
            case coral:
                coralSys.disable();
            case climb:
                climb.disable();
            case algae:
                algae.disable();
            default:
                break; // Don't do anything if currently disabled or in defense state
        }

        // Enable new state
        driverFeedback.updateModeLEDs(newState);
        switch (newState) {
            case coral:
                coralSys.enable();
            case climb:
                climb.enable();
            case algae:
                algae.enable();
            default:
                break; // Don't do anything if currently disabled or in defense state
        }
    }



}
