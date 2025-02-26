package frc.robot;

import static frc.robot.subsystems.vision.VisionConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.stateCommands.AlgaeState;
import frc.robot.commands.stateCommands.CoralState;
import frc.robot.commands.stateCommands.ClimbState;
import frc.robot.commands.stateCommands.DefenseState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSpark;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIOSpark;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmIO;
import frc.robot.subsystems.arm.ArmIOSpark;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;


/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // Subsystems
    private final Drive drive;
    private final Vision vision;
    private final Elevator elevator;
    private final Arm arm;

    // Controller
    private final CommandXboxController driveController = new CommandXboxController(0);
    private final CommandXboxController operatorController = new CommandXboxController(1);
    private final CommandXboxController testController = new CommandXboxController(2);


    // Dashboard inputs
    private final LoggedDashboardChooser<Command> autoChooser;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        switch (Constants.currentMode) {
            case REAL:
                // Real robot, instantiate hardware IO implementations
                drive = new Drive(
                        new GyroIO() {
                        },
                        new ModuleIOSpark(0),
                        new ModuleIOSpark(1),
                        new ModuleIOSpark(2),
                        new ModuleIOSpark(3));
                elevator = new Elevator(new ElevatorIO() {
                });
                arm = new Arm(new ArmIOSpark());
                // Real robot, instantiate hardware IO implementations
                vision = new Vision(
                        drive::addVisionMeasurement,
                        new VisionIOLimelight(camera0Name, drive::getRotation),
                        new VisionIOLimelight(camera1Name, drive::getRotation));
                break;

            case SIM:
                // Sim robot, instantiate physics sim IO implementations
                drive = new Drive(
                        new GyroIO() {
                        },
                        new ModuleIOSim(),
                        new ModuleIOSim(),
                        new ModuleIOSim(),
                        new ModuleIOSim());
                vision = new Vision(
                        drive::addVisionMeasurement,
                        new VisionIOPhotonVisionSim(camera0Name, robotToCamera0, drive::getPose),
                        new VisionIOPhotonVisionSim(camera1Name, robotToCamera1, drive::getPose));

                // Don't instatiate an actual elevator
                elevator = new Elevator(new ElevatorIO() {
                });
                arm = new Arm(new ArmIO() {
                });

                break;

            default:
                // Replayed robot, disable IO implementations
                drive = new Drive(
                        new GyroIO() {
                        },
                        new ModuleIO() {
                        },
                        new ModuleIO() {
                        },
                        new ModuleIO() {
                        },
                        new ModuleIO() {
                        });
                vision = new Vision(drive::addVisionMeasurement, new VisionIO() {
                }, new VisionIO() {
                });
                elevator = new Elevator(new ElevatorIO() {
                });
                arm = new Arm(new ArmIO() {
                });

                break;
        }

        // Set up auto routines
        autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

        // Set up SysId routines
        autoChooser.addOption(
                "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
        autoChooser.addOption(
                "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
        autoChooser.addOption(
                "Drive SysId (Quasistatic Forward)",
                drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        autoChooser.addOption(
                "Drive SysId (Quasistatic Reverse)",
                drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        autoChooser.addOption(
                "Drive SysId (Dynamic Forward)", elevator.sysIdDynamic(SysIdRoutine.Direction.kForward));
        autoChooser.addOption(
                "elevator SysId (Dynamic Reverse)", elevator.sysIdDynamic(SysIdRoutine.Direction.kReverse));
        autoChooser.addOption(
                "elevator SysId (Quasistatic Forward)",
                elevator.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        autoChooser.addOption(
                "elevator SysId (Quasistatic Reverse)",
                elevator.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        autoChooser.addOption(
                "elevator SysId (Dynamic Forward)", elevator.sysIdDynamic(SysIdRoutine.Direction.kForward));
        autoChooser.addOption(
                "elevator SysId (Dynamic Reverse)", elevator.sysIdDynamic(SysIdRoutine.Direction.kReverse));

        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
     * it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        // Default command, normal field-relative drive
        drive.setDefaultCommand(
                DriveCommands.joystickDrive(
                        drive,
                        () -> -driveController.getLeftY(),
                        () -> -driveController.getLeftX(),
                        () -> -driveController.getRightX()));
        // TEST COMMAND
        drive.setDefaultCommand(DriveCommands.driveStraightForwardBack(drive, () -> driveController.getLeftY()));
        // Lock to 0° when A button is held
        driveController
                .a()
                .whileTrue(
                        DriveCommands.joystickDriveAtAngle(
                                drive,
                                () -> -driveController.getLeftY(),
                                () -> -driveController.getLeftX(),
                                () -> new Rotation2d()));

        // Switch to X pattern when X button is pressed
        driveController.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

        // Reset gyro to 0° when B button is pressed
        driveController
                .b()
                .onTrue(
                        Commands.runOnce(
                                () -> drive.setPose(
                                        new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                                drive)
                                .ignoringDisable(true));
        operatorController.a().onTrue(Commands.runOnce(() -> arm.setArmAngleDegrees(0), arm))       ;                           
        operatorController.x().onTrue(Commands.runOnce(() -> arm.setArmAngleDegrees(45), arm))       ; 
        operatorController.y().onTrue(Commands.runOnce(() -> arm.setArmAngleDegrees(90), arm))       ; 
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.get();
    }
}
