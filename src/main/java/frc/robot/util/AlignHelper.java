package frc.robot.util;

import static edu.wpi.first.units.Units.*;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;
import java.util.Map.Entry;
import com.pathplanner.lib.util.FlippingUtil;
import com.pathplanner.lib.util.FlippingUtil.FieldSymmetry;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.kAutoAlign;
import frc.robot.Constants.kAutoAlign.kReef;
import frc.robot.Constants.kAutoAlign.kStation;

public class AlignHelper {
    public static enum kClosestType {
        DISTANCE,
        ROTATION
    }

    public static enum kDirection {
        LEFT,
        RIGHT,
        BOTH
    }

    private static ChassisSpeeds speeds = new ChassisSpeeds();
    private static int timer = kAutoAlign.TIME_ADJUSTMENT_TIMEOUT;

    public static void reset(ChassisSpeeds speeds) {
        AlignHelper.speeds = speeds;
        timer = kAutoAlign.TIME_ADJUSTMENT_TIMEOUT;
    }

    public static Pose2d getClosestReef(Pose2d robotPose) {
        return getClosestReef(robotPose, kClosestType.DISTANCE);
    }

    public static Pose2d getClosestReef(Pose2d robotPose, kClosestType type) {
        return getClosestReef(robotPose, type, kDirection.BOTH);
    }

    public static Pose2d getClosestReef(Pose2d robotPose, kClosestType type, kDirection direction) {
        return calculator(robotPose, type, getBranchPoses(direction));
    }

    public static Pose2d getClosestStation(Pose2d robotPose) {
        return getClosestStation(robotPose, kClosestType.DISTANCE, kDirection.BOTH);
    }

    public static Pose2d getClosestStation(Pose2d robotPose, kClosestType type, kDirection station) {        
        return calculator(robotPose, type, getStationPoses(station));
    }

    public static Pose2d getClosestElement(Pose2d robotPose) {
        return getClosestElement(robotPose, kDirection.BOTH);
    }

    public static Pose2d getClosestElement(Pose2d robotPose, kDirection direction) {
        Collection<Pose2d> poses = getStationPoses(direction);
        poses.addAll(getBranchPoses(direction));

        return calculator(robotPose, kClosestType.DISTANCE, poses);
    }

    private static Collection<Pose2d> getStationPoses(kDirection station) {
        List<Pose2d> poses = new ArrayList<>();
        for (int i = -1; i < 6; i++) {
            if (station == kDirection.LEFT  || station == kDirection.BOTH)
                poses.add(kStation.LEFT_STATION .transformBy(kStation.STATIONS_OFFSET.times( i)));
            
            if (station == kDirection.RIGHT || station == kDirection.BOTH)
                poses.add(kStation.RIGHT_STATION.transformBy(kStation.STATIONS_OFFSET.times(-i)));
        }

        return poses;
    }

    private static Collection<Pose2d> getBranchPoses(kDirection branch) {
        List<Pose2d> poses = new ArrayList<>();
        for (Entry<String, Pose2d> entry : kReef.BRANCHES.entrySet()) {
            if (branch == kDirection.BOTH) {
                poses.add(entry.getValue());
                continue;
            }

            if (branch == kDirection.LEFT  && entry.getKey().endsWith("L"))
                poses.add(entry.getValue());

            if (branch == kDirection.RIGHT && entry.getKey().endsWith("R"))
                poses.add(entry.getValue());
        }

        return poses;
    }

    private static Pose2d calculator(Pose2d robotPose, kClosestType type, Collection<Pose2d> poses) {
        Pose2d estimatedPose = robotPose.plus(
            new Transform2d(
                speeds.vxMetersPerSecond * kAutoAlign.VELOCITY_TIME_ADJUSTEDMENT.in(Seconds),
                speeds.vyMetersPerSecond * kAutoAlign.VELOCITY_TIME_ADJUSTEDMENT.in(Seconds),
                robotPose.getRotation().plus(
                    Rotation2d.fromRadians(speeds.omegaRadiansPerSecond * kAutoAlign.VELOCITY_TIME_ADJUSTEDMENT.in(Seconds))
                )
            )
        );

        if (timer-- <= 0)
            speeds = new ChassisSpeeds();
        
        boolean shouldFlip = DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red;

        FlippingUtil.symmetryType = FieldSymmetry.kRotational;

        double min = -1;
        Pose2d closest = null;

        for (Pose2d branch : poses) {
            Pose2d pose = branch;
            if (shouldFlip) 
                pose = FlippingUtil.flipFieldPose(pose);

            if (type == kClosestType.DISTANCE) {
                double dist = estimatedPose.getTranslation().getDistance(pose.getTranslation());
                if (min == -1 || dist < min) {
                    min = dist;
                    closest = pose;
                }
            } else if (type == kClosestType.ROTATION) {
                double rotation = Math.abs(estimatedPose.getRotation().minus(pose.getRotation()).getRadians());
                if (min == -1 || rotation < min) {
                    min = rotation;
                    closest = pose;
                }
            } else throw new IllegalArgumentException("AlignHelper Recieved an invalid type");
        }

        return closest;
    }
}
