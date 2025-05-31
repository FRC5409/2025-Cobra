package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.kAutoAlign;
import frc.robot.Constants.kAutoAlign.kReef;
import frc.robot.subsystems.drive.Drive;

public class AutoGroups {
    private AutoGroups() {}

    public static Command CLOSE_4_L4(Drive drive) {
        return Commands.sequence(
            DriveCommands.alignToPoint(drive, () -> kReef.BRANCHES.get("FAR_RIGHT.L"), kAutoAlign.autoConfig),
            DriveCommands.alignToPoint(drive, () -> new Pose2d(1.68, 0.90, Rotation2d.fromDegrees(54.60)), kAutoAlign.autoConfig),
            DriveCommands.alignToPoint(drive, () -> kReef.BRANCHES.get("CLOSE_RIGHT.R"), kAutoAlign.autoConfig),
            DriveCommands.alignToPoint(drive, () -> new Pose2d(1.68, 0.90, Rotation2d.fromDegrees(54.60)), kAutoAlign.autoConfig),
            DriveCommands.alignToPoint(drive, () -> kReef.BRANCHES.get("CLOSE_RIGHT.L"), kAutoAlign.autoConfig),
            DriveCommands.alignToPoint(drive, () -> new Pose2d(1.68, 0.90, Rotation2d.fromDegrees(54.60)), kAutoAlign.autoConfig),
            DriveCommands.alignToPoint(drive, () -> kReef.BRANCHES.get("CLOSE.R"), kAutoAlign.autoConfig)
        );
    }
}
