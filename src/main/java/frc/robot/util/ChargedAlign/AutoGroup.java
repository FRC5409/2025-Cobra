package frc.robot.util.ChargedAlign;

import static edu.wpi.first.units.Units.*;

import java.util.function.BiConsumer;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutoGroup {

    private final String name;
    private final SequentialCommandGroup commands;
    private final Pose2d startingPose;

    private static BiConsumer<Boolean, Time> autoTimeConsumer = (interupted, time) -> {};
    private static double startTime;

    public static void setAutoTImeCallback(BiConsumer<Boolean, Time> callBack) {
        autoTimeConsumer = callBack;
    }

    public static void addOptions(SendableChooser<AutoGroup> chooser, AutoGroup... groups) {
        for (AutoGroup group : groups)
            chooser.addOption(group.getName(), group);
    }

    public static void addDefaultOption(SendableChooser<AutoGroup> chooser, AutoGroup group) {
        chooser.setDefaultOption(group.getName(), group);
    }

    public static void addOptions(LoggedDashboardChooser<AutoGroup> chooser, AutoGroup... groups) {
        for (AutoGroup group : groups)
            chooser.addOption(group.getName(), group);
    }

    public static void addDefaultOption(LoggedDashboardChooser<AutoGroup> chooser, AutoGroup group) {
        chooser.addDefaultOption(group.getName(), group);
    }
    
    public AutoGroup(final String name, final Pose2d startingPose) {
        this.name = name;
        this.startingPose = startingPose;

        commands = new SequentialCommandGroup(
            Commands.runOnce(() -> {
                    ChargedAlign.poseResetter.accept(startingPose);
                    startTime = Timer.getFPGATimestamp();
                }
            )
        );
    }

    public AutoGroup withCommands(final Command... commands) {
        this.commands.addCommands(commands);

        return this;
    }

    public String getName() {
        return name;
    }

    public Pose2d getStartingPose() {
        return startingPose;
    }

    public Command build() {
        return commands.finallyDo(interupted -> autoTimeConsumer.accept(
            interupted,
            Seconds.of(
                Timer.getFPGATimestamp() - startTime
            )
        ));
    }
}
