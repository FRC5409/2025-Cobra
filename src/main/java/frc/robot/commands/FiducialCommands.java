package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.kAutoAlign;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.Vision;

public class FiducialCommands {
    public static Command alignToVisibleFiducial(Drive drive, Vision vision) {
        return Commands.race(
                Commands.runEnd(
                        () -> drive.runVelocity(ChassisSpeeds.fromRobotRelativeSpeeds(
                                Math.min(                                                                       // move forward to increase ta
                                        (1-vision.getTargetOffset().z) * kAutoAlign.kFiducialBased.SAGITTAL_AXIS_VELOCITY_FACTOR,
                                        kAutoAlign.MAX_VELOCITY.in(Units.MetersPerSecond)),
                                0,                                                             // do not move sideways
                                Math.min(vision.getTargetOffset().x, kAutoAlign.MAX_ANGULAR_VELOCITY),          // rotate X to face tag
                                drive.getRotation())),
                        drive::stop,
                        drive, vision),
                Commands.waitUntil(() -> vision.getTargetOffset().z >= kAutoAlign.kFiducialBased.AREA_PERCENT_TO_FINISH));
    }
}
