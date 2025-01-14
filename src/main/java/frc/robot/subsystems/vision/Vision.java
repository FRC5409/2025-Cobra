package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

// 5409: The Chargers
// http://github.com/FRC5409

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kVision;
import frc.robot.LimelightHelpers;

public class Vision extends SubsystemBase {
    private final VisionIO io;
    private final VisionInputsAutoLogged inputs;

    public Vision(VisionIO io) {
        this.io = io;
        inputs = new VisionInputsAutoLogged();
    }

    /**
     * Estimates the robot pose given the apriltags on the field
     * 
     * @return Pose2D
     */
    public Pose2d estimatePose(SwerveDrivePoseEstimator poseEstimator) {
        double[] pose = LimelightHelpers.getBotPose(kVision.CAM_NAME);

        if (pose.length != 6)
            throw new IllegalStateException("Invalid pose data received");

        return new Pose2d(pose[0], pose[1], Rotation2d.fromDegrees(pose[5]));
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        io.updateInputs(inputs);
        Logger.processInputs("Vision", inputs);
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}