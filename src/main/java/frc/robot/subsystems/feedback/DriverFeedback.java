package frc.robot.subsystems.feedback;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.feedback.FeedbackConstants.FeedbackType;
import static frc.robot.subsystems.feedback.FeedbackConstants.ledFeedbackTable;
import frc.robot.subsystems.feedback.FeedbackConstants.ledModePatterns;
import static frc.robot.subsystems.feedback.FeedbackConstants.ledModeTable;

public class DriverFeedback extends SubsystemBase {

    private final PWMSparkMax blinkin;

    private final CommandXboxController driverController;

    // Track current mode to be able to back after feedback
    private ledModePatterns currentModePattern = ledModePatterns.disabled;

    public DriverFeedback(CommandXboxController driverController) {
        blinkin = new PWMSparkMax(FeedbackConstants.blinkinPort);

        this.driverController = driverController;
    }

    public void updateModeLEDs(ledModePatterns modePattern) {
        currentModePattern = modePattern;
        Logger.recordOutput("Feedback/LEDMode", modePattern);
        Logger.recordOutput("Feedback/LEDControllerNum", ledModeTable.get(modePattern));
        blinkin.set(ledModeTable.get(modePattern));

        // Vibrate driver controller rapidlly twice on mode change
        Commands.sequence(
                new InstantCommand(this::rumbleOn),
                new WaitCommand(FeedbackConstants.modeChangeVibrateLength),
                new InstantCommand(this::rumbleOff),
                new WaitCommand(FeedbackConstants.modeChangeTimeBetweenVibrations),
                new InstantCommand(this::rumbleOn),
                new WaitCommand(FeedbackConstants.modeChangeVibrateLength),
                new InstantCommand(this::rumbleOff));
    }

    public void giveFeedback(FeedbackType feedbackType) {
        Logger.recordOutput("Logger/FeedbackGiven", feedbackType);
        Commands.sequence(
                new InstantCommand(() -> blinkin.set(ledFeedbackTable.get(feedbackType))),
                new WaitCommand(FeedbackConstants.feedbackLedLength),
                new InstantCommand(() -> blinkin.set(ledModeTable.get(currentModePattern))));

        Commands.sequence(
                new InstantCommand(this::rumbleOn),
                new WaitCommand(FeedbackConstants.feedbackVibrationLength),
                new InstantCommand(this::rumbleOff));
    }

    private void rumbleOn() {
        driverController.setRumble(RumbleType.kBothRumble, FeedbackConstants.vibrationIntensity);
    }

    private void rumbleOff() {
        driverController.setRumble(RumbleType.kBothRumble, 0);
    }

}
