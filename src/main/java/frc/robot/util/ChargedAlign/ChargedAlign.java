package frc.robot.util.ChargedAlign;

import static edu.wpi.first.units.Units.*;

import java.util.function.Consumer;
import java.util.function.Supplier;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class ChargedAlign {

    private static final double timeLoop = 0.02;
    private static Double lastTimestamp;

    private static Supplier<Pose2d> robotPose;
    private static Supplier<ChassisSpeeds> robotSpeeds;
    private static Consumer<ChassisSpeeds> velocityConsumer;

    private static AlignConfig currentConfig;
    private static ProfiledController headingController;

    private ChargedAlign() {}

    public static void configure(Supplier<Pose2d> robotPose, Supplier<ChassisSpeeds> robotRelativeSpeeds, Consumer<ChassisSpeeds> fieldRelativeControl, PIDConstants headingConstants) {
        ChargedAlign.robotPose = robotPose;
        ChargedAlign.robotSpeeds = robotRelativeSpeeds;
        ChargedAlign.velocityConsumer = fieldRelativeControl;

        headingController = new ProfiledController(headingConstants, 0, 0);
        headingController.enableContinuousInput(-Math.PI, Math.PI);
        setConfig(new AlignConfig());
    }

    public static void setConfig(AlignConfig config) {
        currentConfig = config;

        headingController.setContraints(
            config.getMaxAngularVelocity().orElse(DegreesPerSecond.of(360000)).in(RadiansPerSecond),
            config.getMaxAngularAcceleration().orElse(DegreesPerSecondPerSecond.of(360000)).in(RadiansPerSecondPerSecond)
        );
    }

    private static double getMagnitude(ChassisSpeeds speeds) {
        return Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
    }

    /**
     * Calculates rotation difference for rotation2d
     * @param r1 1st rotation2d
     * @param r2 2nd rotation2d
     * @return The angle between them
     */
    private static Angle rotationDifference(Rotation2d r1, Rotation2d r2) {
        double difference = r1.getRadians() - r2.getRadians();

        difference = (difference + Math.PI) % (2 * Math.PI) - Math.PI;

        while (difference < -Math.PI)
            difference += 2 * Math.PI;

        while (difference > Math.PI)
            difference -= 2 * Math.PI;

        return Radians.of(Math.abs(difference));
    }

    public static Command run(Supplier<Pose2d> targetPose, Subsystem driveSubsystem) {
        return run(targetPose, MetersPerSecond.of(0.0), driveSubsystem);
    }

    public static Command run(Supplier<Pose2d> targetPose, LinearVelocity endVelocity, Subsystem driveSubsystem) {
        return Commands.run(() -> {
            Pose2d robot = robotPose.get();
            Pose2d target = targetPose.get();
            double robotSpeed = getMagnitude(robotSpeeds.get());
            double Vf = endVelocity.in(MetersPerSecond);
            double d = robot.getTranslation().getDistance(target.getTranslation());
            
            double maxAccel = currentConfig.getMaxAccelerationMetersPerSecondPerSecond();
            
            double dt;
            if (lastTimestamp == null)
                dt = timeLoop;
            else
                dt = Timer.getFPGATimestamp() - lastTimestamp;
            
            double targetVelocity = Math.sqrt(Math.max(0.0, Vf * Vf + 2 * maxAccel * d));
            
            double maxDeltaV = maxAccel * dt;
            double limitedVelocity = Math.min(robotSpeed + maxDeltaV, targetVelocity);
            
            double estimatedRobotSpeed = robotSpeed * 2.0 + maxDeltaV; // Mult by 2 for some reason...?

            double stoppingDistance = (estimatedRobotSpeed * estimatedRobotSpeed - Vf * Vf) / (2 * maxAccel);

            if (d <= stoppingDistance)
                limitedVelocity = Math.max(robotSpeed - maxDeltaV, Vf);
            
            Rotation2d theta = Rotation2d.fromRadians(Math.atan2(
                target.getY() - robot.getY(),
                target.getX() - robot.getX()
            ));
            
            velocityConsumer.accept(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                    new ChassisSpeeds(
                        limitedVelocity * theta.getCos(),
                        limitedVelocity * theta.getSin(),
                        headingController.calculate(robot.getRotation().getRadians(), target.getRotation().getRadians())
                    ),
                    robot.getRotation()
                )
            );
            
            lastTimestamp = Timer.getFPGATimestamp();
        }, driveSubsystem)
        .until(() -> {
            Pose2d robot = robotPose.get();
            Pose2d target = targetPose.get();

            return robot.getTranslation().getDistance(target.getTranslation()) <= currentConfig.getTranslationToleranceMeters()
                   && rotationDifference(robot.getRotation(), target.getRotation()).lte(currentConfig.getRotationTolerance());
        })
        .beforeStarting(() -> headingController.reset(robotSpeeds.get().omegaRadiansPerSecond))
        .finallyDo(() -> lastTimestamp = null);
    }
}
