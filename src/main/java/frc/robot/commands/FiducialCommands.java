package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.Vision;

public class FiducialCommands {
    public static Command alignToVisibleFiducial(Drive drive, Vision vision) {
        return Commands.race(
                Commands.runEnd(
                        () -> drive.runVelocity(ChassisSpeeds.fromRobotRelativeSpeeds(
                                (int)vision.getTargetOffsetX(), 0, 0, drive.getRotation())),
                        drive::stop,
                        drive, vision),
                Commands.waitUntil(() -> vision.getTargetOffsetX() == 0));
    }
}
