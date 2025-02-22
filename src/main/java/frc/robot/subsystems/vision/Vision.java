package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Alert;

// 5409: The Chargers
// http://github.com/FRC5409

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.kVision;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.RobotSystemTest;
import frc.robot.util.RobotSystemTest.TestResult;

public class Vision extends SubsystemBase {
    private final VisionIO io;
    private final VisionInputsAutoLogged inputs;

    private final Alert alert_discon = new Alert("Limelight appears to be disconnected. (TIMEOUT)", Alert.AlertType.kError);

    public Vision(VisionIO io) {
        this.io = io;
        inputs = new VisionInputsAutoLogged();
        io.setCameraOffset();

        RobotSystemTest.register("Can see target?", () -> 
            inputs.hasTarget ? TestResult.success() 
                             : TestResult.fail("Cannot find target. Is there an AprilTag in front of the robot?"));

        RobotSystemTest.register("Can recognize target?", () -> 
            TestResult.success("Saw target with ID " + inputs.targetId));
    }

    /**
     * Estimates the robot pose given the AprilTags on the field
     * @param drive Drive subsystem to adjust odometry of
     */
    public void addPoseEstimate(Drive drive) {
        PoseEstimate estimate = io.estimatePose(drive);

        if (estimate == null || estimate.tagCount < kVision.FIDUCIAL_TRUST_THRESHOLD)
            return;

        drive.addVisionMeasurement(estimate.pose, estimate.timestampSeconds);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        io.updateInputs(inputs);
        Logger.processInputs("Vision", inputs);

        alert_discon.set(!inputs.isConnected && Constants.currentMode != Constants.Mode.SIM);
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
        io.simulationPeriodic();
    }
}