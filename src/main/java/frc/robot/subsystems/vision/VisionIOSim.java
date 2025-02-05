package frc.robot.subsystems.vision;

import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.subsystems.drive.Drive;

public class VisionIOSim implements VisionIO {
    private final SwerveDriveSimulation sim;

    public VisionIOSim(SwerveDriveSimulation sim) {
        this.sim = sim;
    }

    @Override
    public void updateInputs(VisionInputs inputs) {
        inputs.isConnected = true;
        inputs.fps = 50.0;
    }

    @Override
    public PoseEstimate estimatePose(Drive drive) {
        drive.setPose(sim.getSimulatedDriveTrainPose());

        return null;
    }
}
