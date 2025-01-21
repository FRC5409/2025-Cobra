package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.subsystems.drive.Drive;

public interface VisionIO {
    @AutoLog
    class VisionInputs {
        public double tx = 0;
        public double ty = 0;
        public double ta = 0;
        public boolean hasTarget = false;
        public double targetId = 0;
    }

    default void updateInputs(VisionInputs inputs) {}

    /**
     * Sets the camera offset and return nothing
     */
    default void setCameraOffset() {}

    /**
     * Estimate the current pose using AprilTags
     * @param drive Drive subsystem to get rotation from
     * @return PoseEstimate
     */
    default PoseEstimate estimatePose(Drive drive) {
        return new PoseEstimate();
    }
}
