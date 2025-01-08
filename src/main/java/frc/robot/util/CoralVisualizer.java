package frc.robot.util;

import java.util.ArrayList;
import java.util.function.Supplier;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;

public class CoralVisualizer {
    public static class Coral {
        private static int noteId = 0;

        private Pose3d pose;

        private boolean attached;
        private Supplier<Transform3d> coralOffset;
        private Supplier<Pose2d> robotPose;

        private StructPublisher<Pose3d> posePublisher;

        public Coral(Supplier<Transform3d> coralOffset, Supplier<Pose2d> robotPose) {
            this.coralOffset = coralOffset;
            this.robotPose = robotPose;

            this.pose = new Pose3d(robotPose.get()).transformBy(coralOffset.get());

            this.posePublisher = NetworkTableInstance.getDefault()
                .getStructTopic("Coral-" + (noteId++), Pose3d.struct).publish();

            attached = true;
        }

        public void updatePose() {
            if (!attached) return;

            this.pose = new Pose3d(robotPose.get()).transformBy(coralOffset.get());

            posePublisher.accept(this.pose);
        }

        public Pose3d getPose() {
            return pose;
        }

        public void attach() {
            attached = true;
        }

        public void detach() {
            attached = false;
        }
    }

    private static Supplier<Pose2d> robotPose = null;
    private static ArrayList<Coral> corals = new ArrayList<>();

    public static void configure(Supplier<Pose2d> robotPose) {
        CoralVisualizer.robotPose = robotPose;
    }

    public static Coral addCoral(Supplier<Transform3d> coralOffset) {
        if (robotPose == null) throw new RuntimeException("CoralVisulizer was not configured!");

        Coral coral = new Coral(coralOffset, robotPose);

        corals.add(coral);

        return coral;
    }

    public static void update() {
        for (Coral coral : corals)
            coral.updatePose();
    }
}
