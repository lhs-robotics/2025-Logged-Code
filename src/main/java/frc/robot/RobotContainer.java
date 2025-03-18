package frc.robot;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.auto.AlignmentCommands;
import frc.robot.commands.coral.DelayUntilCoralIntake;
import frc.robot.commands.coral.ReleaseCoral;
import frc.robot.subsystems.algae.Algae;
import frc.robot.subsystems.algae.AlgaeIO;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.climb.ClimbIO;
import frc.robot.subsystems.climb.ClimbIOSpark;
import frc.robot.subsystems.coral.CoralConstants.CoralState;
import frc.robot.subsystems.coral.CoralSystem;
import frc.robot.subsystems.coral.arm.Arm;
import frc.robot.subsystems.coral.arm.ArmIO;
import frc.robot.subsystems.coral.arm.ArmIOSpark;
import frc.robot.subsystems.coral.elevator.Elevator;
import frc.robot.subsystems.coral.elevator.ElevatorIO;
import frc.robot.subsystems.coral.elevator.ElevatorIOSpark;
import frc.robot.subsystems.coral.indexer.Indexer;
import frc.robot.subsystems.coral.indexer.IndexerIO;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSpark;
import frc.robot.subsystems.feedback.DriverFeedback;
import frc.robot.subsystems.vision.Vision;
import static frc.robot.subsystems.vision.VisionConstants.camera0Name;
import static frc.robot.subsystems.vision.VisionConstants.camera1Name;
import static frc.robot.subsystems.vision.VisionConstants.robotToCamera0;
import static frc.robot.subsystems.vision.VisionConstants.robotToCamera1;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;

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
	public final Drive drive;
	private final Vision vision;
	private final CoralSystem coralSys;
	private final DriverFeedback driverFeedback;
	private final Climb climb;
	private final Algae algae;
	// public final StateManager stateManager;

	// Controller
	private final CommandXboxController driveController = new CommandXboxController(0);
	private final CommandXboxController operatorController = new CommandXboxController(1);
	private final CommandXboxController testController = new CommandXboxController(3);

	// Dashboard inputs
	private final LoggedDashboardChooser<Command> autoChooser;

	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */
	public RobotContainer() {
		driverFeedback = new DriverFeedback(operatorController);

		switch (Constants.currentMode) {
			case REAL -> {

				// Replayed robot, disable IO implementations
				drive = new Drive(
						new GyroIOPigeon2(),
						new ModuleIOSpark(0),
						new ModuleIOSpark(1),
						new ModuleIOSpark(2),
						new ModuleIOSpark(3));
				vision = new Vision(drive::addVisionMeasurement, drive::addLocalVisionMeasurement, new VisionIO() {
				}, new VisionIO() {
				});
				coralSys = new CoralSystem(
						new Arm(new ArmIOSpark()),
						new Elevator(new ElevatorIOSpark() {
						}),
						new Indexer(new IndexerIO() {

						}),
						driverFeedback);
				climb = new Climb(new ClimbIOSpark());
				algae = new Algae(new AlgaeIO() {

				});
			}

			// case REAL -> {
			// // Real robot, instantiate hardware IO implementations
			// drive = new Drive(
			// new GyroIO() {
			// },
			// new ModuleIOSpark(0),
			// new ModuleIOSpark(1),
			// new ModuleIOSpark(2),
			// new ModuleIOSpark(3));
			// coralSys = new CoralSystem(
			// new Arm(new ArmIOSpark()),
			// new Elevator(new ElevatorIOSpark()), new Indexer(new IndexerIOSpark()),
			// driverFeedback);
			// vision = new Vision(
			// drive::addVisionMeasurement,
			// new VisionIOLimelight(camera0Name, drive::getRotation),
			// new VisionIOLimelight(camera1Name, drive::getRotation));
			// climb = new Climb(new ClimbIOSpark());
			// algae = new Algae(new AlgaeIOSpark());

			// }

			case SIM -> {
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
						drive::addLocalVisionMeasurement,
						new VisionIOPhotonVisionSim(camera0Name, robotToCamera0,
								drive::getPose),
						new VisionIOPhotonVisionSim(camera1Name, robotToCamera1,
								drive::getPose));

				coralSys = new CoralSystem(
						new Arm(new ArmIO() {
						}),
						new Elevator(new ElevatorIO() {
						}),
						new Indexer(new IndexerIO() {

						}),
						driverFeedback);
				climb = new Climb(new ClimbIO() {

				});
				algae = new Algae(new AlgaeIO() {

				});
			}

			default -> {
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
				vision = new Vision(drive::addVisionMeasurement, drive::addLocalVisionMeasurement, new VisionIO() {
				}, new VisionIO() {
				});
				coralSys = new CoralSystem(
						new Arm(new ArmIO() {
						}),
						new Elevator(new ElevatorIO() {
						}),
						new Indexer(new IndexerIO() {

						}),
						driverFeedback);
				climb = new Climb(new ClimbIO() {
				});
				algae = new Algae(new AlgaeIO() {

				});
			}
		}
		// stateManager = new StateManager(coralSys, climb, algae, driverFeedback);

		// Set up auto routines
		registerAutoCommands();
		autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
		autoChooser.addOption("DumbDriveOffLine", DriveCommands.driveOffLine(drive).withTimeout(2.5)
				.andThen(DriveCommands.joystickDrive(drive, () -> 0, () -> 0, () -> 0)));
		// Set up SysId routines
		autoChooser.addOption(
				"Drive Wheel Radius Characterization",
				DriveCommands.wheelRadiusCharacterization(drive));
		autoChooser.addOption(
				"Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
		autoChooser.addOption(
				"Drive SysId (Quasistatic Forward)",
				drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
		autoChooser.addOption(
				"Drive SysId (Quasistatic Reverse)",
				drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
		autoChooser.addOption(
				"Drive SysId (Dynamic Forward)",
				coralSys.elevator.sysIdDynamic(SysIdRoutine.Direction.kForward));
		autoChooser.addOption(
				"elevator SysId (Dynamic Reverse)",
				coralSys.elevator.sysIdDynamic(SysIdRoutine.Direction.kReverse));
		autoChooser.addOption(
				"elevator SysId (Quasistatic Forward)",
				coralSys.elevator.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
		autoChooser.addOption(
				"elevator SysId (Quasistatic Reverse)",
				coralSys.elevator.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
		autoChooser.addOption(
				"elevator SysId (Dynamic Forward)",
				coralSys.elevator.sysIdDynamic(SysIdRoutine.Direction.kForward));
		autoChooser.addOption(
				"elevator SysId (Dynamic Reverse)",
				coralSys.elevator.sysIdDynamic(SysIdRoutine.Direction.kReverse));

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
		System.out.println("BINDINGS CONFIGURED");
		// Default command, normal field-relative drive
		drive.setDefaultCommand(
				DriveCommands.joystickDrive(
						drive,
						() -> -driveController.getLeftY(),
						() -> -driveController.getLeftX(),
						() -> -driveController.getRightX()));

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

		driveController.rightBumper()
				.whileTrue(Commands.startEnd(() -> climb.climbWellHeldStart(true), () -> climb.stopClimb(), climb));
		driveController.leftBumper()
				.whileTrue(Commands.startEnd(() -> climb.climbWellHeldStart(false), () -> climb.stopClimb(), climb));
		// Reset gyro to 0° when B button is pressed
		driveController
				.b()
				.onTrue(
						Commands.runOnce(
								() -> drive.setPose(
										new Pose2d(drive.getPose()
												.getTranslation(),
												new Rotation2d())),
								drive)
								.ignoringDisable(true));

		// operatorController.y().whileTrue(coralSys.elevator.manualElevatorUpCommand());
		// operatorController.a().whileTrue(coralSys.elevator.manualElevatorDownCommand());
		// operatorController.x().whileTrue(coralSys.arm.dumbManualArmDown());
		// operatorController.b().whileTrue(coralSys.arm.dumbManualArmUp());
		// operatorController.y().onTrue(Commands.runOnce(() ->
		// coralSys.arm.setArmAngleDegrees(0), coralSys.arm));
		// operatorController.x()
		// .onTrue(Commands.runOnce(() -> coralSys.arm.setArmAngleDegrees(45),
		// coralSys.arm));
		// operatorController.y()
		// .onTrue(Commands.runOnce(() -> coralSys.arm.setArmAngleDegrees(90),
		// coralSys.arm));
		// operatorController.y().onTrue(new
		// ParallelCommandGroup(Commands.run(()->coralSys.elevator.setElevatorToLocation(elevatorPositions.kLevel4),
		// coralSys.elevator), new SequentialCommandGroup(new WaitCommand(2),
		// Commands.run(()->coralSys.arm.setArmToPosition(ArmPositions.kLevel4)))));
		// operatorController.rightTrigger(0.75).onTrue(Commands.run(()->
		// coralSys.arm.releaseCoral(), coralSys.arm));
		// operatorController.leftBumper().onTrue(new
		// ParallelCommandGroup(Commands.run(()->coralSys.arm.setArmToPosition(ArmPositions.loadPosition),
		// coralSys.arm),new SequentialCommandGroup(new WaitCommand(2),
		// Commands.run(()->coralSys.elevator.setElevatorToLocation(elevatorPositions.preLoadPosition),
		// coralSys.elevator))));
		// testController.a().whileTrue(coralSys.elevator.manualElevatorUpCommand());
		testController.a().onTrue(Commands.run(() -> coralSys.elevator.setElevatorHeightInches(1), coralSys.elevator));
		testController.b().onTrue(Commands.run(() -> coralSys.elevator.setElevatorHeightInches(3), coralSys.elevator));
		testController.x().onTrue(Commands.run(() -> coralSys.elevator.setElevatorHeightInches(5), coralSys.elevator));
		testController.y().onTrue(Commands.run(() -> coralSys.elevator.setElevatorHeightInches(10), coralSys.elevator));
		testController.rightBumper()
				.onTrue(Commands.run(() -> coralSys.elevator.setElevatorHeightInches(15), coralSys.elevator));
		testController.leftBumper()
				.onTrue(Commands.run(() -> coralSys.elevator.setElevatorHeightInches(20), coralSys.elevator));
		operatorController.a()
				.onTrue(Commands.run(() -> coralSys.elevator.setElevatorHeightInches(30), coralSys.elevator));
		operatorController.b()
				.onTrue(Commands.run(() -> coralSys.elevator.setElevatorHeightInches(35), coralSys.elevator));
		// testController.x().whileTrue(coralSys.arm.dumbManualArmUp());
		// testController.b().whileTrue(coralSys.arm.dumbManualArmDown());
		// testController.leftTrigger(.75).whileTrue(Commands.startEnd(()->coralSys.arm.endAffectorIntakeEnable(),
		// ()->coralSys.arm.endAffectorIntakeDisable(), coralSys.arm));
		// testController.rightTrigger(.75).onTrue(coralSys.arm.releaseCoral());
	}

	private void registerAutoCommands() {
		NamedCommands.registerCommand("AlignReefLeft", AlignmentCommands.alignToLeftReef(drive));
		NamedCommands.registerCommand("AlignReefRight", AlignmentCommands.alignToLeftReef(drive));
		NamedCommands.registerCommand("AlignCoralStation", AlignmentCommands.alignToCoralStation(drive));
		NamedCommands.registerCommand("CoralStateL4", new InstantCommand(() -> coralSys.setCoralState(CoralState.kL4)));
		NamedCommands.registerCommand("CoralStateL3", new InstantCommand(() -> coralSys.setCoralState(CoralState.kL3)));
		NamedCommands.registerCommand("CoralStateL2", new InstantCommand(() -> coralSys.setCoralState(CoralState.kL2)));
		NamedCommands.registerCommand("CoralStateL1", new InstantCommand(() -> coralSys.setCoralState(CoralState.kL1)));
		NamedCommands.registerCommand("ReleaseCoral", new ReleaseCoral(coralSys));
		NamedCommands.registerCommand("DelayUntilCoralIntake", new DelayUntilCoralIntake(coralSys));
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
