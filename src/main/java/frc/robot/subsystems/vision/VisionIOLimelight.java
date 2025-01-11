package frc.robot.subsystems.vision;

import frc.robot.LimelightHelpers;
import frc.robot.Constants.kVision;

public class VisionIOLimelight implements VisionIO {

    @Override
    public void updateInputs(VisionInputs inputs) {
        inputs.tx = LimelightHelpers.getTX(kVision.HOSTNAME);
        inputs.ty = LimelightHelpers.getTY(kVision.HOSTNAME);
        inputs.ta = LimelightHelpers.getTA(kVision.HOSTNAME);
        inputs.hasTarget = LimelightHelpers.getTV(kVision.HOSTNAME);
    }

}
