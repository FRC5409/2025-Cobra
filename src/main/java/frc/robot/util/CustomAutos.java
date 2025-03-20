package frc.robot.util;

import static edu.wpi.first.units.Units.*;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.stream.Stream;
import org.json.simple.parser.ParseException;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.CommandUtil;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;
import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class CustomAutos {
    
    private CustomAutos() {}

    public abstract static class Request {

        private Request() {}

        protected abstract Optional<ArrayList<Request>> getChildren();

        protected abstract Command build();

        protected abstract Command buildMirrored();
    }

    public static class SequentialRequest extends Request {

        private final ArrayList<Request> requests;

        public SequentialRequest(Request... requests) {
            this.requests = new ArrayList<>(Arrays.asList(requests));
        }

        @Override
        protected Optional<ArrayList<Request>> getChildren() {
            return Optional.of(requests);
        }

        @Override
        protected Command build() {
            return new SequentialCommandGroup(requests.stream().map(r -> r.build()).toArray(Command[]::new));
        }
        
        @Override
        protected Command buildMirrored() {
            return new SequentialCommandGroup(requests.stream().map(r -> r.buildMirrored()).toArray(Command[]::new));
        } 
    }

    public static class ParallelRequest extends Request {

        private final ArrayList<Request> requests;

        public ParallelRequest(Request... requests) {
            this.requests = new ArrayList<>(Arrays.asList(requests));
        }

        @Override
        protected Optional<ArrayList<Request>> getChildren() {
            return Optional.of(requests);
        }

        @Override
        protected Command build() {
            return new ParallelCommandGroup(requests.stream().map(r -> r.build()).toArray(Command[]::new));
        }
        
        @Override
        protected Command buildMirrored() {
            return new ParallelCommandGroup(requests.stream().map(r -> r.buildMirrored()).toArray(Command[]::new));
        } 
    }

    public static class RaceRequest extends Request {

        private final ArrayList<Request> requests;

        public RaceRequest(Request... requests) {
            this.requests = new ArrayList<>(Arrays.asList(requests));
        }

        @Override
        protected Optional<ArrayList<Request>> getChildren() {
            return Optional.of(requests);
        }

        @Override
        protected Command build() {
            return Commands.race(requests.stream().map(r -> r.build()).toArray(Command[]::new));
        }

        @Override
        protected Command buildMirrored() {
            return Commands.race(requests.stream().map(r -> r.buildMirrored()).toArray(Command[]::new));
        } 
    }

    public static class DeadlineRequest extends Request {

        private final Request deadlineRequest;
        private final ArrayList<Request> requests;

        public DeadlineRequest(Request deadline, Request... requests) {
            this.deadlineRequest = deadline;
            this.requests = new ArrayList<>(Arrays.asList(requests));
        }

        @Override
        protected Optional<ArrayList<Request>> getChildren() {
            return Optional.of(requests);
        }

        @Override
        protected Command build() {
            return Commands.deadline(deadlineRequest.build(), requests.stream().map(r -> r.build()).toArray(Command[]::new));
        }

        @Override
        protected Command buildMirrored() {
            return Commands.deadline(deadlineRequest.build(), requests.stream().map(r -> r.buildMirrored()).toArray(Command[]::new));
        }
    }
    
    public static class WaitRequest extends Request {
        
        private final Time time;

        public WaitRequest(double seconds) {
            this(Seconds.of(seconds));
        }

        public WaitRequest(Time time) {
            this.time = time;
        }

        @Override
        protected Optional<ArrayList<Request>> getChildren() {
            return Optional.empty();
        }

        @Override
        protected Command build() {
            return new WaitCommand(time);
        }

        @Override
        protected Command buildMirrored() {
            return build();
        }
    }

    public static class CommandRequest extends Request {

        private final Command command;

        public CommandRequest(String commandName) {
            this(NamedCommands.getCommand(commandName));
        }

        public CommandRequest(Command command) {
            this.command = CommandUtil.wrappedEventCommand(command);
        }

        @Override
        protected Optional<ArrayList<Request>> getChildren() {
            return Optional.empty();
        }

        @Override
        protected Command build() {
            return CommandUtil.wrappedEventCommand(command);
        }

        @Override
        protected Command buildMirrored() {
            return build();
        }
        
    }

    public static class WaitUntilRequest extends Request {

        private final BooleanSupplier supplier;

        public WaitUntilRequest(BooleanSupplier supplier) {
            this.supplier = supplier;
        }

        @Override
        protected Optional<ArrayList<Request>> getChildren() {
            return Optional.empty();
        }

        @Override
        protected Command build() {
            return new WaitUntilCommand(supplier);
        }

        @Override
        protected Command buildMirrored() {
            return build();
        }
    }

    public static class ConditionalRequest extends Request {

        private final Request onTrue, onFalse;
        private final BooleanSupplier supplier;

        public ConditionalRequest(Request onTrue, Request onFalse, BooleanSupplier condition) {
            this.onTrue = onTrue;
            this.onFalse = onFalse;

            this.supplier = condition;
        }

        @Override
        protected Optional<ArrayList<Request>> getChildren() {
            return Optional.of(new ArrayList<>(List.of(onTrue, onFalse)));
        }

        @Override
        protected Command build() {
            return new ConditionalCommand(
                onTrue.build(),
                onFalse.build(),
                supplier
            );
        }

        @Override
        protected Command buildMirrored() {
            return new ConditionalCommand(
                onTrue.buildMirrored(),
                onFalse.buildMirrored(),
                supplier
            );
        }
    }

    public static class PathRequest extends Request {

        private final PathPlannerPath path;

        public PathRequest(String pathName) throws FileVersionException, IOException, ParseException {
            this(PathPlannerPath.fromPathFile(pathName));
        }

        public PathRequest(PathPlannerPath path) {
            this.path = path;
        }

        @Override
        protected Optional<ArrayList<Request>> getChildren() {
            return Optional.empty();
        }

        @Override
        protected Command build() {
            return AutoBuilder.followPath(path);
        }

        @Override
        protected Command buildMirrored() {
            return AutoBuilder.followPath(path.mirrorPath());
        }
    }

    public static class ResetRequest extends Request {
        
        private final Consumer<Pose2d> poseConsumer;
        private final Pose2d resetPose;

        public ResetRequest(Consumer<Pose2d> consumer, Pose2d pose) {
            poseConsumer = consumer;
            resetPose = pose;
        }

        @Override
        protected Optional<ArrayList<Request>> getChildren() {
            return Optional.empty();
        }

        @Override
        protected Command build() {
            return Commands.either(
                    Commands.runOnce(() -> poseConsumer.accept(FlippingUtil.flipFieldPose(resetPose)))
                        .ignoringDisable(true),
                    Commands.runOnce(() -> poseConsumer.accept(resetPose))
                        .ignoringDisable(true),
                    AutoBuilder::shouldFlip
            );
                    
        }

        @Override
        protected Command buildMirrored() {
            return Commands.either(
                    Commands.runOnce(() -> poseConsumer.accept(FieldMirror.mirrorPose(FlippingUtil.flipFieldPose(resetPose))))
                        .ignoringDisable(true),
                    Commands.runOnce(() -> poseConsumer.accept(FieldMirror.mirrorPose(resetPose)))
                        .ignoringDisable(true),
                    AutoBuilder::shouldFlip
            );
        }
    }

    public static Command buildAuto(ResetRequest reset, Request... requests) {
        return buildAuto(false, reset, requests);
    }

    public static Command buildAuto(boolean mirror, ResetRequest reset, Request... requests) {
        Request[] commands = Stream.concat(Stream.of(reset), Arrays.stream(requests)).toArray(Request[]::new);
        if (mirror)
            return new SequentialRequest(commands).buildMirrored();
        else
            return new SequentialRequest(commands).build();
    }
}
