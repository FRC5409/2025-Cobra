package frc.robot.subsystems.vision;

import java.io.IOException;

import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.subsystems.drive.Drive;

public class VisionIOSim implements VisionIO {
    private final SwerveDriveSimulation sim_drive;
    private final VisionSystemSim sim_vision;

    private final SimCameraProperties prop = new SimCameraProperties();

    private static volatile VisionIOSim globalThis = null;

    public VisionIOSim(SwerveDriveSimulation sim) {
        if (globalThis != null) throw new IllegalArgumentException("You cannot create more than one VisionIOSim!");

        this.sim_drive = sim;
        this.sim_vision = new VisionSystemSim("PV-simsystem");

        try {
            AprilTagFieldLayout tagLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.kDefaultField.m_resourceFile);
            sim_vision.addAprilTags(tagLayout);
        } catch (IOException e) {
            throw new RuntimeException("Failed to load PV AprilTag layout '" + AprilTagFields.kDefaultField + "': " + AprilTagFields.kDefaultField.m_resourceFile);
        }

        prop.setCalibration(1280, 600, Rotation2d.fromDegrees(97.6524449259));
        prop.setCalibError(0.25, 0.08);
        prop.setFPS(20);
        prop.setAvgLatencyMs(35);
        prop.setLatencyStdDevMs(5);

        PhotonCamera camera = new PhotonCamera("PV-simcamera");
        PhotonCameraSim cameraSim = new PhotonCameraSim(camera, prop);

        sim_vision.addCamera(cameraSim, new Transform3d(new Translation3d(0.171919, 0, 0.629752), new Rotation3d()));

        globalThis = this;
    }

    @Override
    public void updateInputs(VisionInputs inputs) {
        inputs.isConnected = true;
        inputs.fps         = 20;
        inputs.prxLatency  = 35;
    }

    @Override
    public PoseEstimate estimatePose(Drive drive) {
        // TODO: implement
        return new PoseEstimate();
    }

    @Override
    public void simulationPeriodic() {
        sim_vision.update(sim_drive.getSimulatedDriveTrainPose());
        Logger.recordOutput("Simulation/Vision/RobotPose", sim_vision.getRobotPose());
    }
}
