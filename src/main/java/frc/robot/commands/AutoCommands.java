package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.kAutoAlign;
import frc.robot.Constants.kAutoAlign.kReef;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.AlignHelper;
import frc.robot.util.DebugCommand;
import frc.robot.util.WaitThen;
import frc.robot.util.AlignHelper.kClosestType;
import frc.robot.util.AlignHelper.kDirection;

public class AutoCommands {
    public static enum kReefPosition {
        FAR,
        FAR_LEFT,
        FAR_RIGHT,
        CLOSE,
        CLOSE_LEFT,
        CLOSE_RIGHT
    }

    private static final PathConstraints CONSTRAINTS = kAutoAlign.PATH_FIND_CONSTRAINTS;
    private static final Distance DISTANCE_TO_PREPARE = Meters.of(1.5);
    private static final LinearVelocity REEF_END_VELOCITY = MetersPerSecond.of(0.5);
    
    private static GenericEntry isLeftEntry;
    private static kReefPosition target = kReefPosition.CLOSE_LEFT;

    private AutoCommands() {}

    public static void setupNodeChooser() {
        for (kReefPosition reef : kReefPosition.values()) {
            DebugCommand.register(reef.name(), Commands.runOnce(() -> target = reef));
        }

        isLeftEntry = Shuffleboard.getTab("Debug").add("isLeft", false).getEntry();
    }

    public static Command pathfindTo(Drive drive, Supplier<Pose2d> pose, LinearVelocity endVelocity) {
        return AutoBuilder.pathfindToPose(pose.get(), CONSTRAINTS, endVelocity);
    }

    public static Command pathFindToNearestStation(Drive drive) {
        return pathfindTo(
            drive, 
            () -> AlignHelper.getClosestStation(
                drive.getPose(), 
                kClosestType.DISTANCE, 
                kDirection.BOTH
            ),
            MetersPerSecond.of(0.0)
        ).alongWith(
            new WaitThen(
                () -> {
                    Pose2d station = AlignHelper.getClosestStation(
                        drive.getPose(), 
                        kClosestType.DISTANCE, 
                        kDirection.BOTH
                    );

                    return drive.getPose().getTranslation().getDistance(station.getTranslation()) <= DISTANCE_TO_PREPARE.in(Meters);
                }, 
                Commands.print("PREPARE INTAKE")
                .until(() -> true) // TODO: When pickup is achieved
            )
        ).beforeStarting(() -> AlignHelper.reset(drive.getFieldRelativeSpeeds()));
    }

    public static Command pathFindToReef(Drive drive, Supplier<kReefPosition> reef) {
        return pathfindTo(
            drive,
            () -> kReef.TARGETS.getOrDefault(
                reef.get(), 
                new Pose2d()
            ),
            REEF_END_VELOCITY
        );
    }

    public static Command alignToBranch(Drive drive, Supplier<kDirection> direction) {
        return DriveCommands.alignToPoint(
            drive, 
            () -> AlignHelper.getClosestReef(drive.getPose(), kClosestType.DISTANCE, direction.get())
        ).beforeStarting(() -> AlignHelper.reset(drive.getFieldRelativeSpeeds()));
    }

    public static Supplier<Command> telopAutoCommand(Drive drive) {
        return () -> Commands.sequence(
            pathFindToNearestStation(drive),
            pathFindToReef(drive, () -> target),
            alignToBranch(drive, () -> (isLeftEntry.getBoolean(false) ? kDirection.LEFT : kDirection.RIGHT))
        ).repeatedly();
    }
}
