package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.kAutoAlign;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.Vision;
import org.opencv.core.Point;

public class FiducialCommands {
    /**
     * Aligns to the active fiducial, with an optional offset
     * @param drive drivetrain
     * @param vision vision module
     * @return runnable command that aligns to the fiducial given the parameters
     */
    public static Command alignToVisibleFiducial(Drive drive, Vision vision) {
        return Commands.race(
                Commands.runEnd(
                        () -> {
                            Point target = vision.getTargetOffset();
                            drive.runVelocity(ChassisSpeeds.fromRobotRelativeSpeeds(
                                    Math.min(                                                                       // move forward to increase ta
                                            (1 - target.y) * kAutoAlign.kFiducialBased.SAGITTAL_AXIS_VELOCITY_FACTOR,
                                            kAutoAlign.MAX_VELOCITY.in(Units.MetersPerSecond)),
                                    0,                                                             // do not move sideways
                                    Math.min(target.x, kAutoAlign.MAX_ANGULAR_VELOCITY),                            // rotate X to face tag
                                    drive.getRotation()));
                        },
                        drive::stop,
                        drive, vision),
                Commands.waitUntil(() -> vision.getTargetOffset().y >= kAutoAlign.kFiducialBased.FIDUCIAL_AREA_THRESHOLD));
    }
}
