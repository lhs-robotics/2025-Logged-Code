package frc.robot.commands.auto;

import java.util.Set;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import static edu.wpi.first.units.Units.Inches;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Field;
import frc.robot.Robot;
import frc.robot.subsystems.drive.Drive;

public class AlignmentCommands {
    /**
     * Creates a command to drive the robot to the nearest left-sdie reef pole from its current position
     * @return driveToPoseCommand to drive to the nearest reef pole on your side
     */
    public static Command alignToLeftReef(Drive drive) {
        return Commands.defer(
            () -> {
                Pose2d reefPose = drive.getPose().nearest(Robot.onRedAlliance() ? Field.redReefListLeft : Field.blueReefListLeft);
                Translation2d reefOffset = new Translation2d(Field.reefOffsetDistance, Inches.of(0)).rotateBy(reefPose.getRotation());
                return drive.driveToPoseCommand(
                        reefPose.getMeasureX().plus(reefOffset.getMeasureX()),
                        reefPose.getMeasureY().plus(reefOffset.getMeasureY()),
                        reefPose.getRotation().plus(Rotation2d.fromDegrees(180))
                ); 
            },
            Set.of(drive)
        ).withName("Align Left Reef");
    }

    /**
     * Creates a command to drive the robot to the nearest right-side reef pole from its current position
     * @return driveToPoseCommand to drive to the nearest reef pole on your side
     */
    public static Command alignToRightReef(Drive drive) {
        return Commands.defer(
            () -> {
                Pose2d reefPose = drive.getPose().nearest(Robot.onRedAlliance() ? Field.redReefListRight : Field.blueReefListRight);
                Translation2d reefOffset = new Translation2d(Field.reefOffsetDistance, Inches.of(0)).rotateBy(reefPose.getRotation());
                return drive.driveToPoseCommand(
                        reefPose.getMeasureX().plus(reefOffset.getMeasureX()),
                        reefPose.getMeasureY().plus(reefOffset.getMeasureY()),
                        reefPose.getRotation().plus(Rotation2d.fromDegrees(180))
                ); 
            },
            Set.of(drive)
        ).withName("Align Right Reef");
    }

    /**
     * Creates a command to drive the robot to the nearest coral station to it
     * @return driveToPoseCommand to drive to the nearest station on your side
     */
    public static Command alignToCoralStation(Drive drive) {
        return Commands.defer(
            () -> {
                Pose2d stationPose = drive.getPose().nearest(Robot.onRedAlliance() ? Field.redCoralStationList : Field.blueCoralStationList);
                Translation2d stationOffset = new Translation2d(Field.stationOffsetDistance, Inches.of(0)).rotateBy(stationPose.getRotation());
                return drive.driveToPoseCommand(            
                        stationPose.getMeasureX().plus(stationOffset.getMeasureX()),
                        stationPose.getMeasureY().plus(stationOffset.getMeasureY()),
                        stationPose.getRotation().plus(Rotation2d.fromDegrees(180))
                );
            }, 
            Set.of(drive)
        ).withName("Align Coral Station");
    }

}
