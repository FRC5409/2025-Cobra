package frc.robot.util;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;
import com.pathplanner.lib.util.FlippingUtil;
import com.pathplanner.lib.util.FlippingUtil.FieldSymmetry;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.kAutoAlign.kReef;
import frc.robot.Constants.kAutoAlign.kStation;

public class AlignHelper {
    public static enum kClosestType {
        DISTANCE,
        ROTATION
    }

    public static Pose2d getClosestReef(Pose2d robotPose) {
        return getClosestReef(robotPose, kClosestType.DISTANCE);
    }

    public static Pose2d getClosestReef(Pose2d robotPose, kClosestType type) {
        return calculator(robotPose, type, kReef.TARGETS.values());
    }

    public static Pose2d getClosestStation(Pose2d robotPose) {
        return getClosestStation(robotPose, kClosestType.DISTANCE);
    }

    public static Pose2d getClosestStation(Pose2d robotPose, kClosestType type) {
        List<Pose2d> poses = new ArrayList<>();
        for (int i = 0; i <= 6; i++) {
            poses.add(kStation.LEFT_STATION .transformBy(kStation.STATIONS_OFFSET.times(i)));
            poses.add(kStation.RIGHT_STATION.transformBy(kStation.STATIONS_OFFSET.times(i)));
        }
        
        return calculator(robotPose, type, poses);
    }

    private static Pose2d calculator(Pose2d robotPose, kClosestType type, Collection<Pose2d> poses) {
        boolean shouldFlip = DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red;

        FlippingUtil.symmetryType = FieldSymmetry.kRotational;

        double min = -1;
        Pose2d closest = null;

        for (Pose2d branch : poses) {
            Pose2d pose = branch;
            if (shouldFlip) 
                pose = FlippingUtil.flipFieldPose(pose);

            if (type == kClosestType.DISTANCE) {
                double dist = robotPose.getTranslation().getDistance(pose.getTranslation());
                if (min == -1 || dist < min) {
                    min = dist;
                    closest = pose;
                }
            } else if (type == kClosestType.ROTATION) {
                double rotation = Math.abs(robotPose.getRotation().minus(pose.getRotation()).getRadians());
                if (min == -1 || rotation < min) {
                    min = rotation;
                    closest = pose;
                }
            } else throw new IllegalArgumentException("AlignHelper Recieved an invalid type");
        }

        return closest;
    }
}
