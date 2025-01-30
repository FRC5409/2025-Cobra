package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.kAutoAlign;
import frc.robot.Constants.kAutoAlign.kReef;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.AlignHelper;
import frc.robot.util.CaseCommand;
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
    
    private static Supplier<Boolean> scoreRight;
    private static LoggedDashboardChooser<kDirection> stationChooser;
    private static kReefPosition target = kReefPosition.CLOSE_LEFT;

    private AutoCommands() {}

    public static void setupNodeChooser() {
        for (kReefPosition reef : kReefPosition.values()) {
            DebugCommand.register(reef.name(), Commands.runOnce(() -> target = reef));
        }

        scoreRight = DebugCommand.putNumber("Score Right", false);
        stationChooser = new LoggedDashboardChooser<>("Station Select");

        stationChooser.addDefaultOption("Automatic", kDirection.BOTH );
        stationChooser.addOption(       "Left",      kDirection.LEFT );
        stationChooser.addOption(       "Right",     kDirection.RIGHT);
    }

    public static Command pathfindTo(Drive drive, Supplier<Pose2d> targetPose) {
        List<Pose2d> poses = new ArrayList<>();
        poses.addAll(AlignHelper.getStationPoses(kDirection.BOTH));
        poses.addAll(kReef.TARGETS.values());

        BooleanSupplier[] suppliers = new BooleanSupplier[poses.size()];
        Command[] commands = new Command[poses.size()];

        for (int i = 0; i < poses.size(); i++) {
            final Pose2d pose = poses.get(i);

            suppliers[i] = () -> pose.equals(targetPose.get());

            commands[i] = AutoBuilder.pathfindToPoseFlipped(pose, CONSTRAINTS);
        }

        return CaseCommand.build(suppliers, commands, Commands.print("ERROR WITH AUTO-TELE"));
    }

    public static Command pathFindToNearestStation(Drive drive) {
        return pathfindTo(
            drive, 
            () -> AlignHelper.getClosestStation(
                drive.getBlueSidePose(), 
                kClosestType.DISTANCE, 
                stationChooser.get()
            )
            // , MetersPerSecond.of(0.0)
        ).alongWith(
            new WaitThen(
                () -> {
                    Pose2d station = AlignHelper.getClosestStation(
                        drive.getBlueSidePose(), 
                        kClosestType.DISTANCE, 
                        kDirection.BOTH
                    );

                    return drive.getBlueSidePose().getTranslation().getDistance(station.getTranslation()) <= DISTANCE_TO_PREPARE.in(Meters);
                }, 
                Commands.print("PREPARE INTAKE")
                .until(() -> true) // TODO: When pickup is achieved
            )
        ).beforeStarting(() -> AlignHelper.reset(drive.getFieldRelativeSpeeds()))
        .andThen(Commands.waitSeconds(0.5));
    }

    public static Command pathFindToReef(Drive drive, Supplier<kReefPosition> reef) {
        return pathfindTo(
            drive,
            () -> kReef.TARGETS.getOrDefault(
                reef.get(), 
                new Pose2d()
            )
            // , REEF_END_VELOCITY
        );
    }

    public static Command alignToBranch(Drive drive, Supplier<kDirection> direction) {
        return DriveCommands.alignToPoint(
            drive, 
            () -> AlignHelper.getClosestReef(drive.getBlueSidePose(), kClosestType.DISTANCE, direction.get())
        ).beforeStarting(() -> AlignHelper.reset(drive.getFieldRelativeSpeeds()));
    }

    public static Command telopAutoCommand(Drive drive) {
        return Commands.sequence(
            pathFindToNearestStation(drive).unless(() -> false), // TODO: Has coral
            pathFindToReef(drive, () -> target),
            alignToBranch(drive, () -> (scoreRight.get() ? kDirection.RIGHT : kDirection.LEFT)),
            Commands.print("Score!"),
            Commands.waitSeconds(0.5)
        ).repeatedly();
    }
}
