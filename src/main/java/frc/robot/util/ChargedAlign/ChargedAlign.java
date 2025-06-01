package frc.robot.util.ChargedAlign;

import static edu.wpi.first.units.Units.*;

import java.util.function.Consumer;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class ChargedAlign {

    private static final double deltaT = 0.02;
    private static Double lastTimestamp;

    private static Supplier<Pose2d> robotPose;
    private static Supplier<ChassisSpeeds> robotSpeeds;
    private static Consumer<ChassisSpeeds> velocityConsumer;

    private static AlignConfig currentConfig;

    protected static Consumer<Pose2d> poseResetter;

    private static Consumer<LinearVelocity> targetVelocityConsumer = velo   -> {};
    private static Consumer<AlignConfig>    currentConfigConsumer  = config -> {};
    private static Consumer<Pose2d>         targetConsumer         = target -> {};

    private ChargedAlign() {}

    public static void configure(Supplier<Pose2d> robotPose, Supplier<ChassisSpeeds> robotRelativeSpeeds, Consumer<ChassisSpeeds> fieldRelativeControl, Consumer<Pose2d> poseReset) {
        if (ChargedAlign.robotPose != null) throw new IllegalAccessError("Charged Align was already configured!");

        ChargedAlign.robotPose = robotPose;
        ChargedAlign.robotSpeeds = robotRelativeSpeeds;
        ChargedAlign.velocityConsumer = fieldRelativeControl;
        ChargedAlign.poseResetter = poseReset;

        setConfig(new AlignConfig());
    }

    public static void setConfig(AlignConfig config) {
        currentConfig = config;
    }

    public static void setLogCallback(Consumer<LinearVelocity> targetVelocity, Consumer<AlignConfig> currentConfig, Consumer<Pose2d> target) {
        targetVelocityConsumer = targetVelocity;
        currentConfigConsumer = currentConfig;
        targetConsumer = target;
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
        return Radians.of(MathUtil.angleModulus(r2.getRadians() - r1.getRadians()));
    }

    public static Command run(Supplier<Pose2d> targetPose, Subsystem driveSubsystem) {
        return new RunUntilCommand(() -> {
            AlignConfig config = currentConfig.copy();

            ChassisSpeeds speeds = robotSpeeds.get();
            Pose2d robot = robotPose.get();
            Pose2d target = targetPose.get();
            double robotSpeed = getMagnitude(speeds);
            double Vf = config.getEndVelocityMetersPerSecond();
            double d = robot.getTranslation().getDistance(target.getTranslation());

            d = MathUtil.applyDeadband(d, config.getTranslationToleranceMeters());
            
            double maxAccel = config.getMaxAccelerationMetersPerSecondPerSecond();
            double maxVelo = config.getMaxVelocityMetersPerSecond();
            
            double dt;
            if (lastTimestamp == null)
                dt = deltaT;
            else
                dt = Timer.getFPGATimestamp() - lastTimestamp;
            
            double targetVelocity = Math.min(Math.sqrt(Math.max(0.0, Vf * Vf + 2.0 * maxAccel * d)), maxVelo);
            
            double maxDeltaV = maxAccel * dt;

            double limitedVelocity = Math.min(robotSpeed + maxDeltaV, targetVelocity);
            
            double estimatedRobotSpeed = Math.min(robotSpeed + maxDeltaV, maxVelo) * 2.0;

            double stoppingDistance = (estimatedRobotSpeed * estimatedRobotSpeed - Vf * Vf) / (2.0 * maxAccel);

            if (d <= stoppingDistance) {
                double percentToTarget = 1.0 - (d / stoppingDistance);
                Logger.recordOutput("C/Percent", percentToTarget);
                percentToTarget = MathUtil.clamp(percentToTarget, 0.0, 1.0);
            
                double adjustedAccel = MathUtil.clamp(maxAccel * 1.1 * percentToTarget, 0.20 * maxAccel, maxAccel * 1.1);
                double deltaV = adjustedAccel * dt;
            
                if (robotSpeed >= Vf)
                    limitedVelocity = Math.max(robotSpeed - deltaV, Vf);
            }
            
            Logger.recordOutput("C/RobotSpeed", robotSpeed);
            Logger.recordOutput("C/Stopping", stoppingDistance);
            Logger.recordOutput("C/Distnce", d);
            Logger.recordOutput("C/SlowingDown", d <= stoppingDistance);
            
            Rotation2d theta = Rotation2d.fromRadians(Math.atan2(
                target.getY() - robot.getY(),
                target.getX() - robot.getX()
            ));

            double maxAngularVelo = config.getMaxAngularVelocity().in(RadiansPerSecond);
            double maxAngularAccel = config.getMaxAngularAcceleration().in(RadiansPerSecondPerSecond);

            double currentOmega = speeds.omegaRadiansPerSecond;
            double eTheta = rotationDifference(robot.getRotation(), target.getRotation()).in(Radians);
            double omegaF = 0.0; // End angular velo 0 - maybe a future command

            double angularTargetVelocity = Math.sqrt(Math.max(0.0, omegaF * omegaF + 2 * maxAngularAccel * Math.abs(eTheta)));
            double maxAngularDeltaV = maxAngularAccel * dt;
            double limitedAngularVelocity = Math.min(Math.abs(currentOmega) + maxAngularDeltaV, angularTargetVelocity);

            double estimatedOmega = currentOmega * 2.0 + maxAngularDeltaV;

            double angularStoppingDistance = (estimatedOmega * estimatedOmega - omegaF * omegaF) / (2 * maxAngularAccel);

            if (Math.abs(eTheta) <= angularStoppingDistance) {
                double percentToTarget = 1.0 - (Math.abs(eTheta) / angularStoppingDistance);
                percentToTarget = MathUtil.clamp(percentToTarget, 0.0, 1.0);
            
                double adjustedAngularAccel = MathUtil.clamp(maxAngularAccel * percentToTarget, 0.2 * maxAngularAccel, maxAngularAccel);
                double angularDeltaV = adjustedAngularAccel * dt;
            
                limitedAngularVelocity = Math.max(Math.abs(currentOmega) - angularDeltaV, omegaF);
            }
            
            
            limitedAngularVelocity = Math.copySign(
                Math.min(Math.abs(limitedAngularVelocity), maxAngularVelo),
                eTheta
            );

            if (Math.abs(eTheta) <= config.getRotationTolerance().in(Radians))
                limitedAngularVelocity = 0.0;

            velocityConsumer.accept(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                    new ChassisSpeeds(
                        limitedVelocity * theta.getCos(),
                        limitedVelocity * theta.getSin(),
                        limitedAngularVelocity
                    ),
                    robot.getRotation()
                )
            );

            targetVelocityConsumer.accept(MetersPerSecond.of(limitedVelocity));
            currentConfigConsumer.accept(config);
            targetConsumer.accept(target);

            boolean ends = d <= config.getTranslationToleranceMeters()
                && Math.abs(rotationDifference(robot.getRotation(), target.getRotation()).in(Radians)) <= config.getRotationTolerance().in(Radians);
            
            lastTimestamp = Timer.getFPGATimestamp();

            return ends;
        }, driveSubsystem)
        .finallyDo(() -> lastTimestamp = null);
    }
}
