package frc.robot.subsystems.vision;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.kVision;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.subsystems.drive.Drive;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {
    private final VisionIO io;
    private final VisionInputsAutoLogged inputs;

//    private final GenericEntry sTagCount;
//    private final Field2d sEstimatedPose;

    private final Alert disconnectedAlert = new Alert("Limelight appears to be disconnected. (TIMEOUT)", Alert.AlertType.kError);

    public Vision(VisionIO io) {
        this.io = io;
        inputs = new VisionInputsAutoLogged();

        io.setCameraOffset();

//        ShuffleboardTab tab = Shuffleboard.getTab("Vision");
//        sTagCount = tab.add("Tag Count", 0).getEntry();
//        sEstimatedPose = new Field2d();
//        tab.add("Estimated Pose", sEstimatedPose);
    }

    /**
     * Estimates the robot pose given the AprilTags on the field
     * @param drive Drive subsystem to adjust odometry of
     */
    public void addPoseEstimate(Drive drive) {
        PoseEstimate estimate = io.estimatePose(drive);

        if (estimate == null)
            return;

//        sTagCount.setInteger(estimate.tagCount);
//        sEstimatedPose.setRobotPose(estimate.pose);

        if (estimate.tagCount < kVision.FIDUCIAL_TRUST_THRESHOLD)
            return;

        drive.addVisionMeasurement(estimate.pose, estimate.timestampSeconds, VecBuilder.fill(.5, .5, 9999999));
    }

    /**
     * Relative offset values to the active target
     * @param dx    difference to target in degrees
     * @param a     percentage of FOV filled by target
     */
    public record VisionOffset(double dx, double a) {}

    public VisionOffset getTargetOffset() {
        return new VisionOffset(inputs.tx, inputs.ta);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        io.updateInputs(inputs);
        Logger.processInputs("Vision", inputs);

        disconnectedAlert.set(!inputs.isConnected && Constants.currentMode != Constants.Mode.SIM);
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}