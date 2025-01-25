package frc.robot.commands;

import com.pathplanner.lib.config.PIDConstants;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.kAutoAlign;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.Vision;

public class FiducialCommands {
    private static final PIDConstants SAGITTAL_PID = new PIDConstants(1,0.0001,0.001);
    private static final PIDConstants ANGULAR_PID = new PIDConstants(1,0.0001,0);

    /**
     * Aligns to the active fiducial, with an optional offset
     * @param drive drivetrain
     * @param vision vision module
     * @return runnable command that aligns to the fiducial given the parameters
     */
    public static Command alignToVisibleFiducial(Drive drive, Vision vision) {
        ProfiledPIDController sagittalController =
                new ProfiledPIDController(
                        SAGITTAL_PID.kP, SAGITTAL_PID.kI, SAGITTAL_PID.kD,
                        new TrapezoidProfile.Constraints(
                                kAutoAlign.MAX_SAGITTAL_VELOCITY.in(Units.MetersPerSecond),
                                kAutoAlign.MAX_ACCELERATION.in(Units.MetersPerSecondPerSecond)));

        ProfiledPIDController angularController =
                new ProfiledPIDController(
                        ANGULAR_PID.kP, ANGULAR_PID.kI, ANGULAR_PID.kD,
                        new TrapezoidProfile.Constraints(kAutoAlign.MAX_ANGULAR_VELOCITY, kAutoAlign.MAX_ANGULAR_ACCELERATION));
        angularController.enableContinuousInput(-Math.PI, Math.PI);

        return Commands.parallel(
                Commands.runOnce(() -> {
                    ChassisSpeeds cs = drive.getChassisSpeeds();
                    sagittalController.reset(vision.getTargetOffset().a(), cs.vxMetersPerSecond);
                    angularController.reset(drive.getRotation().getRadians(), cs.omegaRadiansPerSecond);
                }),
                Commands.runEnd(
                        () -> {
                            Vision.VisionOffset t = vision.getTargetOffset();
                            drive.runVelocity(ChassisSpeeds.fromRobotRelativeSpeeds(
                                    sagittalController.calculate(t.a(), kAutoAlign.FIDUCIAL_AREA_GOAL),   // move sagittally proportionally to ta
                                    0,
                                    angularController.calculate(drive.getRotation().getDegrees(), t.dx()), // rotate X to face tag
                                    drive.getRotation()));
                        },
                        drive::stop,
                        drive, vision
                ).onlyWhile(() -> vision.getTargetOffset().a() < kAutoAlign.FIDUCIAL_AREA_GOAL));
    }
}
