package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.kVision;
import frc.robot.subsystems.drive.Drive;

public class VisionIOLimelight implements VisionIO {

    @Override
    public void updateInputs(VisionInputs inputs) {
        inputs.tx = LimelightHelpers.getTX(kVision.CAM_NAME);
        inputs.ty = LimelightHelpers.getTY(kVision.CAM_NAME);
        inputs.ta = LimelightHelpers.getTA(kVision.CAM_NAME);
        inputs.hasTarget = LimelightHelpers.getTV(kVision.CAM_NAME);
        inputs.targetId = LimelightHelpers.getFiducialID(kVision.CAM_NAME);
    }

    @Override
    public void setCameraOffset() {
        LimelightHelpers.setCameraPose_RobotSpace(kVision.CAM_NAME, 0.171919, 0, 0.629752, 0, 0, 0);
    }

    @Override
    public LimelightHelpers.PoseEstimate estimatePose(Drive drive) {
        Rotation2d rot = drive.getRotation();
        LimelightHelpers.SetRobotOrientation(Constants.kVision.CAM_NAME, rot.getDegrees(),
                drive.getChassisSpeeds().omegaRadiansPerSecond, 0, 0, 0, 0);
        return LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(Constants.kVision.CAM_NAME);
    }
}
