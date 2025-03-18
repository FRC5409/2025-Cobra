package frc.robot.commands;

import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Optional;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class Autos {

    public static class Auto {
        private final String name;
        private final boolean mirrorable;

        private final SequentialCommandGroup[] groups;

        public Auto(String name, boolean mirrorable) {
            if (name == null) throw new NullPointerException();

            this.name = name;
            this.mirrorable = mirrorable;

            groups = new SequentialCommandGroup[mirrorable ? 2 : 1];

            for (int i = 0; i < groups.length; i++)
                groups[i] = new SequentialCommandGroup();
        }

        private final PathPlannerPath mirrorPath(PathPlannerPath path) {
            return new PathPlannerPath(
                path.getWaypoints()
                    .stream()
                    .map(o -> o.flip())
                    .toList(), 
                path.getRotationTargets(),
                path.getPointTowardsZones(),
                path.getConstraintZones(),
                path.getEventMarkers(),
                path.getGlobalConstraints(),
                path.getIdealStartingState(),
                path.getGoalEndState(),
                false
            );
        }

        public String getName() {
            return name;
        }

        public boolean isMirrorable() {
            return mirrorable;
        }

        public Auto andThen(Command... command) {
            for (SequentialCommandGroup group : groups)
                group.addCommands(command);

            return this;
        }

        public Auto andThen(PathPlannerPath... paths) {
            for (PathPlannerPath path : paths) {
                groups[0].addCommands(AutoBuilder.followPath(path));
                if (mirrorable)
                    groups[1].addCommands(AutoBuilder.followPath(mirrorPath(path)));
            }

            return this;
        }

        public Auto alongWith(Command... command) {
            for (SequentialCommandGroup group : groups)
                group.addCommands(Commands.parallel(command));

            return this;
        }

        public Auto deadlineFor(Command... command) {
            for (SequentialCommandGroup group : groups)
                group.addCommands(Commands.deadline(group, command));

            return this;
        }

        public Auto raceWith(Command... command) {
            for (SequentialCommandGroup group : groups)
                group.addCommands(Commands.race(command));

            return this;
        }

        public Pair<Command, Optional<Command>> build() {
            if (mirrorable)
                return new Pair<Command,Optional<Command>>(groups[0], Optional.of(groups[1]));
            else
                return new Pair<Command,Optional<Command>>(groups[0], Optional.empty());
        }
    }

    private Autos() {}

    private static final HashMap<String, PathPlannerPath> paths;
    static {
        paths = new HashMap<>();
        for (String auto : AutoBuilder.getAllAutoNames()) {
            try {
                for (PathPlannerPath path : PathPlannerAuto.getPathGroupFromAutoFile(auto)) {
                    paths.computeIfAbsent(path.name, p -> {
                        try {
                            return PathPlannerPath.fromPathFile(p);
                        } catch (FileVersionException | IOException | ParseException e) {
                            System.out.println("Couldn't load path: " + path.name);
                            return null;
                        }
                    });
                }
            } catch (IOException | ParseException e) {
                System.out.println("Coulnd't load paths from auto: " + auto);
            }
        }
    }

    private static PathPlannerPath getPath(String pathName) {
        PathPlannerPath path = paths.getOrDefault(pathName, null);
        if (path == null)
            throw new NullPointerException("Couldn't get path: " + pathName);

        return path;
    }
    
    public static SendableChooser<Command> buildChooser() {
        ArrayList<Auto> autos = new ArrayList<>();

        SendableChooser<Command> chooser = new SendableChooser<>();

        for (Auto auto : autos) {
            Pair<Command, Optional<Command>> built = auto.build();
            if (auto.isMirrorable()) {
                String autoName = auto.getName().replace("[M]", "");
                chooser.addOption("{L} - " + autoName, built.getFirst());
                chooser.addOption("{R} - " + autoName, built.getSecond().get());
            } else {
                chooser.addOption(auto.getName(), built.getFirst());
            }
        }

        return chooser;
    }
}
