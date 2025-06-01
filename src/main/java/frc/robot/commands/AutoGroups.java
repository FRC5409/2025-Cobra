package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.kAutoAlign;
import frc.robot.Constants.kAutoAlign.kReef;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.ChargedAlign.AlignConfig;
import frc.robot.util.ChargedAlign.AutoGroup;

public class AutoGroups {

    private AutoGroups() {}

    public static AutoGroup CLOSE_4_L4(Drive drive) {
        return new AutoGroup("CLOSE_L4", new Pose2d( 7.06, 2.92, Rotation2d.fromDegrees(180))).withCommands(
            DriveCommands.alignToPoint(drive, () -> kReef.BRANCHES.get("FAR_RIGHT.L"), kAutoAlign.autoConfig),
            Commands.waitSeconds(0.5),
            DriveCommands.alignToPoint(drive, () -> new Pose2d(4.04, 2.36, Rotation2d.fromDegrees(120.0)),
                new AlignConfig(
                    MetersPerSecond.of(4.56),
                    MetersPerSecondPerSecond.of(20.0),
                    Centimeters.of(5.00),
                    Degrees.of(5.00),
                    MetersPerSecond.of(4.56)
                )
            ),
            DriveCommands.alignToPoint(drive, () -> new Pose2d(1.638, 0.749, Rotation2d.fromDegrees(54)), kAutoAlign.autoConfig),
            Commands.waitSeconds(0.5),
            DriveCommands.alignToPoint(drive, () -> kReef.BRANCHES.get("CLOSE_RIGHT.R"), kAutoAlign.autoConfig),
            Commands.waitSeconds(0.5),
            DriveCommands.alignToPoint(drive, () -> new Pose2d(1.638, 0.749, Rotation2d.fromDegrees(54)), kAutoAlign.autoConfig),
            Commands.waitSeconds(0.5),
            DriveCommands.alignToPoint(drive, () -> kReef.BRANCHES.get("CLOSE_RIGHT.L"), kAutoAlign.autoConfig),
            Commands.waitSeconds(0.5),
            DriveCommands.alignToPoint(drive, () -> new Pose2d(1.638, 0.749, Rotation2d.fromDegrees(54)), kAutoAlign.autoConfig),
            Commands.waitSeconds(0.5),
            DriveCommands.alignToPoint(drive, () -> kReef.BRANCHES.get("CLOSE.R"), kAutoAlign.autoConfig)
        );
    }

    public static AutoGroup TEST_ALIGN(Drive drive) {
        return new AutoGroup("TEST", new Pose2d( 7.06, 2.92, Rotation2d.fromDegrees(180))).withCommands(
            DriveCommands.alignToPoint(drive, () -> new Pose2d(3.81, 1.28, Rotation2d.fromDegrees(90)), kAutoAlign.autoConfig)
        );
    }
}
