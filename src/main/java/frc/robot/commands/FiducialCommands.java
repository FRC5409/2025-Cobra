package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.Vision;

public class FiducialCommands {
    public Command alignToVisibleFiducial(Drive drive, Vision vision) {
        return Commands.race(
                Commands.runEnd(
                        () -> DriveCommands.joystickDrive(drive, vision::getTargetOffsetX, () -> 0, () -> 0),
                        drive::stop,
                        drive, vision),
                Commands.waitUntil(() -> vision.getTargetOffsetX() == 0));
    }
}
