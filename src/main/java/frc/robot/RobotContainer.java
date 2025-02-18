// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.io.IOException;
import java.util.function.BiFunction;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.json.simple.parser.ParseException;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import frc.robot.Constants.Mode;
import frc.robot.Constants.ScoringLevel;
import frc.robot.Constants.kArmPivot;
import frc.robot.Constants.kAuto;
import frc.robot.Constants.kDrive;
import frc.robot.commands.AutoCommands;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.AutoCommands.kReefPosition;
import frc.robot.commands.scoring.IdleCommand;
import frc.robot.commands.scoring.RemoveAlgae;
import frc.robot.commands.scoring.ScoreCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.arm.ArmPivot;
import frc.robot.subsystems.arm.ArmPivotIO;
import frc.robot.subsystems.arm.ArmPivotIOSim;
import frc.robot.subsystems.arm.ArmPivotIOTalonFX;
import frc.robot.Constants.kElevator;
import frc.robot.Constants.kEndEffector;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Elevator.ElevatorIO;
import frc.robot.subsystems.Elevator.ElevatorIOSim;
import frc.robot.subsystems.Elevator.ElevatorIOTalonFX;
import frc.robot.subsystems.collector.EndEffector;
import frc.robot.subsystems.collector.EndEffectorIO;
import frc.robot.subsystems.collector.EndEffectorIOSim;
import frc.robot.subsystems.collector.EndEffectorIOTalonFx;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.GyroIOSim;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOSim;
import frc.robot.util.AlignHelper;
import frc.robot.util.WaitThen;
import frc.robot.util.AutoTimer;
import frc.robot.util.CaseCommand;
import frc.robot.util.ControlBoard;
import frc.robot.util.DebugCommand;
import frc.robot.util.OpponentRobot;
import frc.robot.util.AlignHelper.kClosestType;
import frc.robot.util.AlignHelper.kDirection;
import frc.robot.util.ControlBoard.kButton;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */

public class RobotContainer {
    // Subsystems
    protected final Drive       sys_drive;
    protected final Vision      sys_vision;
    protected final Elevator    sys_elevator;
    protected final EndEffector sys_endEffector;
    protected final ArmPivot    sys_armPivot;

    public static SwerveDriveSimulation simConfig;

    // Commands
    protected final Command telopAutoCommand;
  
    private ScoringLevel selectedScoringLevel = ScoringLevel.LEVEL4;

    // Controller
    private final CommandXboxController primaryController   = new CommandXboxController(0);
    private final CommandXboxController secondaryController = new CommandXboxController(1);
    private final ControlBoard          controlBoard        = new ControlBoard(2);
    private final CommandXboxController testController      = new CommandXboxController(4);

    public static boolean isTelopAuto = false;

    private boolean removeAlgae = false;

    @SuppressWarnings("unused")
    private static final boolean runOpponent = false
        && Constants.currentMode == Mode.SIM;

    // Dashboard inputs
    protected final LoggedDashboardChooser<Command> autoChooser;
    protected final Supplier<Boolean> runTelop;

    // Alerts
    private final Alert primaryDisconnectedAlert = new Alert(
            "Primary Controller Disconnected!",
            AlertType.kError);
    private final Alert secondaryDisconnectedAlert = new Alert(
            "Secondary Controller Disconnected!",
            AlertType.kError);
    private final Alert controlDisconnectedAlert = new Alert(
            "Control Board Disconnected!",
            AlertType.kError);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {

        DriverStation.silenceJoystickConnectionWarning(true);

        switch (Constants.currentMode) {
            case REAL -> {
                // Real robot, instantiate hardware IO implementations
                sys_vision = new Vision(new VisionIOLimelight());
                sys_drive =
                    new Drive(
                        new GyroIOPigeon2(),
                        new ModuleIOTalonFX(TunerConstants.FrontLeft),
                        new ModuleIOTalonFX(TunerConstants.FrontRight),
                        new ModuleIOTalonFX(TunerConstants.BackLeft),
                        new ModuleIOTalonFX(TunerConstants.BackRight),
                        sys_vision);
              
                sys_armPivot = new ArmPivot(new ArmPivotIOTalonFX(kArmPivot.FALCON_ID, kArmPivot.CANCODER_ID));
                sys_elevator = new Elevator(new ElevatorIOTalonFX(kElevator.MAIN_MOTOR_ID, kElevator.FOLLOWER_MOTOR_ID));
                sys_endEffector = new EndEffector(new EndEffectorIOTalonFx(kEndEffector.ENDEFFECTOR_MOTOR_ID));
            }
            case SIM -> {
                // Sim robot, instantiate physics sim IO implementations
                final DriveTrainSimulationConfig driveConfig = DriveTrainSimulationConfig.Default()
                    .withGyro(COTS.ofPigeon2())
                    .withRobotMass(kDrive.ROBOT_FULL_MASS)
                    .withTrackLengthTrackWidth(Meters.of(0.578), Meters.of(0.578))
                    .withBumperSize(Meters.of(0.881), Meters.of(0.881))
                    .withSwerveModule(
                        COTS.ofMark4i(
                            DCMotor.getKrakenX60Foc(1),
                            DCMotor.getKrakenX60Foc(1),
                            1.20,
                            2
                        )
                    );
              
                sys_armPivot = new ArmPivot(new ArmPivotIOSim());
                sys_elevator = new Elevator(new ElevatorIOSim());
                sys_endEffector = new EndEffector(new EndEffectorIOSim());
                
                simConfig = new SwerveDriveSimulation(
                    driveConfig,
                    new Pose2d(3, 3, new Rotation2d())
                );

                SimulatedArena.getInstance().addDriveTrainSimulation(simConfig);
                SimulatedArena.getInstance().resetFieldForAuto();

                sys_vision = new Vision(new VisionIOSim(simConfig));
                sys_drive =
                    new Drive(
                        new GyroIOSim(simConfig.getGyroSimulation()),
                        new ModuleIOSim(simConfig.getModules()[0]),
                        new ModuleIOSim(simConfig.getModules()[1]),
                        new ModuleIOSim(simConfig.getModules()[2]),
                        new ModuleIOSim(simConfig.getModules()[3]),
                        sys_vision);

                if (runOpponent) {
                    final OpponentRobot sys_opponent = 
                        new OpponentRobot(new Pose2d(3, 3, Rotation2d.fromDegrees(0.0)));
                    sys_opponent.setDefaultCommand(sys_opponent.joystickDrive(secondaryController));
                }
            }
            default -> {
                // Replayed robot, disable IO implementations
                sys_vision = new Vision(new VisionIO() {});
                sys_drive =
                    new Drive(
                        new GyroIO() {},
                        new ModuleIO() {},
                        new ModuleIO() {},
                        new ModuleIO() {},
                        new ModuleIO() {},
                        sys_vision);
                sys_armPivot = new ArmPivot(new ArmPivotIO() {});
                sys_elevator = new Elevator(new ElevatorIO(){});
                sys_endEffector = new EndEffector(new EndEffectorIO() {});
            }
        }

        AutoCommands.setupNodeChooser();
        registerCommands();

        // Commands
        telopAutoCommand = AutoCommands.telopAutoCommand(
            sys_drive,
            sys_elevator,
            sys_armPivot,
            sys_endEffector,
            getLevelSelectorCommand(true),
            () -> removeAlgae,
            () -> secondaryController.getHID().getPOV() != -1
        ).onlyWhile(() -> isTelopAuto);

        // Set up auto routines
        autoChooser = new LoggedDashboardChooser<>("Auto Choices");
        autoChooser.addDefaultOption("None", Commands.none());

        for (String auto : AutoBuilder.getAllAutoNames()) {
            if (auto.endsWith("[M]")) {
                String autoName = auto.replace("[M]", "");
                autoChooser.addOption("{L} - " + autoName, new PathPlannerAuto(auto, false));
                autoChooser.addOption("{R} - " + autoName, new PathPlannerAuto(auto, true ));
            } else {
                autoChooser.addOption(auto, new PathPlannerAuto(auto));
            }
        }

        // Set up SysId routines
        autoChooser.addOption(
                "Drive Wheel Radius Characterization",
                DriveCommands.wheelRadiusCharacterization(sys_drive));
        autoChooser.addOption(
                "Drive Simple FF Characterization",
                DriveCommands.feedforwardCharacterization(sys_drive));
        autoChooser.addOption(
                "Drive SysId (Quasistatic Forward)",
                sys_drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        autoChooser.addOption(
                "Drive SysId (Quasistatic Reverse)",
                sys_drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        autoChooser.addOption(
                "Drive SysId (Dynamic Forward)",
                sys_drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
        autoChooser.addOption(
                "Drive SysId (Dynamic Reverse)",
                sys_drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

                runTelop = DebugCommand.putNumber("Run Telop Auto", false);

        if (kAuto.RESET_ODOM_ON_CHANGE)
            autoChooser
                .getSendableChooser()
                .onChange(path -> resetPose());

        SmartDashboard.putData(
            "Reset",
            Commands.runOnce(() -> resetPose()).ignoringDisable(true));

        // Configure the button bindings
        configureButtonBindings();

        // Controller Alerts
        new Trigger(() -> !primaryController.isConnected())
                .onTrue(
                        Commands.runOnce(() -> {
                            secondaryController.setRumble(RumbleType.kBothRumble, 0.50);
                            primaryDisconnectedAlert.set(true);
                        })
                                .ignoringDisable(true)
                                .andThen(
                                        new WaitThen(
                                                Seconds.of(0.5),
                                                Commands.runOnce(
                                                        () -> secondaryController
                                                                .setRumble(RumbleType.kBothRumble,
                                                                        0.0)))
                                                .ignoringDisable(true)))
                .onFalse(
                        Commands.runOnce(() -> primaryDisconnectedAlert.set(false))
                                .ignoringDisable(true));

        new Trigger(() -> !secondaryController.isConnected())
                .onTrue(
                        Commands.runOnce(() -> {
                            primaryController.setRumble(RumbleType.kBothRumble, 0.50);
                            secondaryDisconnectedAlert.set(true);
                        })
                                .ignoringDisable(true)
                                .andThen(
                                        new WaitThen(
                                                Seconds.of(0.5),
                                                Commands.runOnce(
                                                        () -> primaryController
                                                                .setRumble(RumbleType.kBothRumble,
                                                                        0.0)))
                                                .ignoringDisable(true)))
                .onFalse(
                        Commands.runOnce(() -> secondaryDisconnectedAlert.set(false))
                                .ignoringDisable(true));

        new Trigger(() -> !controlBoard.isConnected())
                .onTrue(
                    Commands.runOnce(() -> {
                        primaryController.setRumble(RumbleType.kBothRumble, 0.50);
                        controlDisconnectedAlert.set(true);
                    })
                            .ignoringDisable(true)
                            .andThen(
                                    new WaitThen(
                                            Seconds.of(0.5),
                                            Commands.runOnce(
                                                    () -> primaryController
                                                            .setRumble(RumbleType.kBothRumble,
                                                                    0.0)))
                                            .ignoringDisable(true)))
            .onFalse(
                    Commands.runOnce(() -> controlDisconnectedAlert.set(false))
                            .ignoringDisable(true));

        // TODO: fix on real robot
        new Trigger(DriverStation::isDSAttached).onTrue(
            Commands.waitSeconds(1.0).andThen(
                Commands.runOnce(() -> {
                    primaryDisconnectedAlert.set(!primaryController.isConnected());
                    secondaryDisconnectedAlert.set(!secondaryController.isConnected());
                    controlDisconnectedAlert.set(!controlBoard.isConnected());

                    resetPose();
                }).ignoringDisable(true)
            )
        );
    }

    public void updateScoringPosition() {
        if (!secondaryController.isConnected()) return;

        if (Math.hypot(secondaryController.getRightX(), secondaryController.getRightY()) < 0.6) return;

        Angle targetAngle = Radians.of(Math.atan2(secondaryController.getRightY(), secondaryController.getRightX()));

        double degrees = targetAngle.in(Degrees);
        if (degrees > -120 && degrees <= -60)
            AutoCommands.target = kReefPosition.CLOSE;
        else if (degrees > -60 && degrees <= 0)
            AutoCommands.target = kReefPosition.CLOSE_LEFT;
        else if (degrees > 0 && degrees <= 60)
            AutoCommands.target = kReefPosition.FAR_LEFT;
        else if (degrees > 60 && degrees <= 120)
            AutoCommands.target = kReefPosition.FAR;
        else if (degrees > 120 && degrees <= 180)
            AutoCommands.target = kReefPosition.FAR_RIGHT;
        else
            AutoCommands.target = kReefPosition.CLOSE_RIGHT;

        Logger.recordOutput("Scoring Position", AutoCommands.target);
    }

    public void updateSim() {
        SimulatedArena.getInstance().simulationPeriodic();
        Logger.recordOutput("Simulation/RobotPose", simConfig.getSimulatedDriveTrainPose());
        Logger.recordOutput(
                "Simulation/Algae", SimulatedArena.getInstance().getGamePiecesArrayByType("Algae"));
        Logger.recordOutput(
                "Simulation/Coral", SimulatedArena.getInstance().getGamePiecesArrayByType("Coral"));
    }

    private void resetPose() {
        Pose2d startingPose = getStartingPose();
        if (Constants.currentMode == Mode.SIM)
            simConfig.setSimulationWorldPose(startingPose);

        sys_drive.setPose(startingPose);
    }

    private Command getLevelSelectorCommand(boolean ends) {
        ScoringLevel[] levels = ScoringLevel.values();
        BooleanSupplier[] conditionals = new BooleanSupplier[levels.length];
        Command[] commands = new Command[levels.length];

        for (int i = 0; i < conditionals.length; i++) {
            final int index = i;
            conditionals[i] = () -> levels[index] == selectedScoringLevel;
            commands[i] = new ScoreCommand(sys_elevator, sys_armPivot, sys_endEffector, levels[index], DriveCommands::isAligned);
        }

        return CaseCommand.buildSelector(
            conditionals, 
            commands, 
            Commands.print("Error: Couldn't find selected level to go to")
        );
    }

    @AutoLogOutput(key = "Odometry/StartingPose")
    public Pose2d getStartingPose() {
        String autoName = autoChooser.getSendableChooser().getSelected();

        boolean mirror = false;
        if ((mirror = autoName.startsWith("{R}")) || autoName.startsWith("{L}")) {
            autoName = autoName.substring(6) + "[M]";
        }

        if (autoName.equals("None"))
            return new Pose2d();

        try {
            PathPlannerPath path = PathPlannerAuto.getPathGroupFromAutoFile(autoName)
                    .get(0);
    
            if (mirror)
                path = path.mirrorPath();
                    
            Pose2d pose = path.getStartingHolonomicPose().get();

            return AutoBuilder.shouldFlip() ? FlippingUtil.flipFieldPose(pose) : pose;
        } catch (IOException | ParseException e) {
            return new Pose2d();
        }
    }

    private void registerCommands() {
        // DEBUG COMMANDS
        DebugCommand.register("Score L1", new ScoreCommand(sys_elevator, sys_armPivot, sys_endEffector, ScoringLevel.LEVEL1, DriveCommands::isAligned));
        DebugCommand.register("Score L2", new ScoreCommand(sys_elevator, sys_armPivot, sys_endEffector, ScoringLevel.LEVEL2, DriveCommands::isAligned));
        DebugCommand.register("Score L3", new ScoreCommand(sys_elevator, sys_armPivot, sys_endEffector, ScoringLevel.LEVEL3, DriveCommands::isAligned));
        DebugCommand.register("Score L4", new ScoreCommand(sys_elevator, sys_armPivot, sys_endEffector, ScoringLevel.LEVEL4, DriveCommands::isAligned));

        // NAMED COMMANDS
        NamedCommands.registerCommand(
                "ALIGN_LEFT",
                new ConditionalCommand(
                    DriveCommands.alignToPoint(sys_drive,
                        () -> AlignHelper.getClosestBranch(
                            sys_drive.getBlueSidePose(),
                            kClosestType.DISTANCE,
                            kDirection.LEFT
                        )
                    ),
                    DriveCommands.alignToPoint(sys_drive,
                        () -> AlignHelper.getClosestBranch(
                            sys_drive.getBlueSidePose(),
                            kClosestType.DISTANCE,
                            kDirection.RIGHT
                        )
                    ),
                    () -> !autoChooser.getSendableChooser().getSelected().startsWith("{R}")
                ).withTimeout(1.5)
            );

        NamedCommands.registerCommand(
                "ALIGN_RIGHT",
                new ConditionalCommand(
                    DriveCommands.alignToPoint(sys_drive,
                        () -> AlignHelper.getClosestBranch(
                            sys_drive.getBlueSidePose(),
                            kClosestType.DISTANCE,
                            kDirection.RIGHT
                        )
                    ),
                    DriveCommands.alignToPoint(sys_drive,
                        () -> AlignHelper.getClosestBranch(
                            sys_drive.getBlueSidePose(),
                            kClosestType.DISTANCE,
                            kDirection.LEFT
                        )
                    ),
                    () -> !autoChooser.getSendableChooser().getSelected().startsWith("{R}")
                ).withTimeout(1.5)
            );

        NamedCommands.registerCommand("IDLE", new IdleCommand(sys_elevator, sys_armPivot, sys_endEffector));

        NamedCommands.registerCommand("L1", new ScoreCommand(sys_elevator, sys_armPivot, sys_endEffector, ScoringLevel.LEVEL1, DriveCommands::isAligned));
        NamedCommands.registerCommand("L2", new ScoreCommand(sys_elevator, sys_armPivot, sys_endEffector, ScoringLevel.LEVEL2, DriveCommands::isAligned));
        NamedCommands.registerCommand("L3", new ScoreCommand(sys_elevator, sys_armPivot, sys_endEffector, ScoringLevel.LEVEL3, DriveCommands::isAligned));
        NamedCommands.registerCommand("L4", new ScoreCommand(sys_elevator, sys_armPivot, sys_endEffector, ScoringLevel.LEVEL4, DriveCommands::isAligned));
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        // Default command, normal field-relative drive
        sys_drive.setDefaultCommand(
            DriveCommands.joystickDrive(
                sys_drive,
                () -> -primaryController.getLeftY(),
                () -> -primaryController.getLeftX(),
                () -> -(primaryController.getRightTriggerAxis() - primaryController.getLeftTriggerAxis())
            )
        );

        // Reset gyro to 0° when Start button is pressed
        primaryController
                .start()
                .onTrue(
                        Commands.runOnce(
                                () -> sys_drive.setPose(
                                        new Pose2d(sys_drive.getPose()
                                                .getTranslation(),
                                                new Rotation2d())),
                                sys_drive).ignoringDisable(true));

        primaryController
            .back()
            .or(() -> secondaryController.getHID().getBackButton())
                .whileFalse(
                    Commands.runOnce(() -> isTelopAuto = !isTelopAuto)
                        .andThen(telopAutoCommand)
                );

        // primaryController.rightBumper()
        //     .and(() -> !isTelopAuto)
        //     .onTrue( Commands.runOnce(() -> sys_drive.coastMode()).ignoringDisable(true))
        //     .onFalse(Commands.runOnce(() -> sys_drive.brakeMode()).ignoringDisable(true));

        primaryController.a()
            .onTrue(getLevelSelectorCommand(false))
            .onFalse(
                new ConditionalCommand(
                    Commands.waitSeconds(0.2),
                    sys_endEffector.runUntilCoralNotDetected(3), 
                    () -> Constants.currentMode == Mode.SIM
                ).andThen(
                    new IdleCommand(sys_elevator, sys_armPivot, sys_endEffector)
                )
            );

        primaryController.x()
            .onTrue(new RemoveAlgae(sys_elevator, sys_armPivot, sys_endEffector, selectedScoringLevel == ScoringLevel.LEVEL3 ? ScoringLevel.LEVEL3_ALGAE : ScoringLevel.LEVEL2_ALGAE))
            .onFalse(new IdleCommand(sys_elevator, sys_armPivot, sys_endEffector, kEndEffector.ALGAE_VOLTAGE));

        primaryController.b()
            .onTrue(new ScoreCommand(sys_elevator, sys_armPivot, sys_endEffector, ScoringLevel.PROCESSOR, () -> true))
            .onFalse(new IdleCommand(sys_elevator, sys_armPivot, sys_endEffector));

        // TODO: Look into barge/create barge command
        // primaryController.y()
        //     .onTrue(new ScoreCommand(sys_elevator, sys_armPivot, sys_endEffector, ScoringLevel.BARGE, () -> true));

        // Reset gyro to 0° when Start button is pressed
        primaryController
                .start()
                .onTrue(
                        Commands.runOnce(
                                () -> sys_drive.setPose(
                                        new Pose2d(sys_drive.getPose()
                                                .getTranslation(),
                                                new Rotation2d())),
                                sys_drive).ignoringDisable(true));

        primaryController
            .leftBumper()
            .and(() -> !isTelopAuto)
                .whileTrue(
                    Commands.parallel(
                        DriveCommands.alignToPoint(
                            sys_drive, 
                            () -> AlignHelper.getClosestBranch(sys_drive.getBlueSidePose(), kClosestType.DISTANCE, kDirection.LEFT)
                        ).beforeStarting(() -> AlignHelper.reset(sys_drive.getFieldRelativeSpeeds()))
                        .alongWith(Commands.waitUntil(() -> sys_elevator.getPosition().gte(ScoringLevel.LEVEL4.elevatorSetpoint.minus(Centimeters.of(5))))),
                        getLevelSelectorCommand(false)
                    ).andThen(
                        new IdleCommand(sys_elevator, sys_armPivot, sys_endEffector)
                    )
                ).onFalse(new IdleCommand(sys_elevator, sys_armPivot, sys_endEffector));

        primaryController
            .rightBumper()
            .and(() -> !isTelopAuto)
                .whileTrue(
                    Commands.parallel(
                        DriveCommands.alignToPoint(
                            sys_drive, 
                            () -> AlignHelper.getClosestBranch(sys_drive.getBlueSidePose(), kClosestType.DISTANCE, kDirection.RIGHT)
                        ).beforeStarting(() -> AlignHelper.reset(sys_drive.getFieldRelativeSpeeds())),
                        getLevelSelectorCommand(false)
                    ).andThen(
                        new IdleCommand(sys_elevator, sys_armPivot, sys_endEffector)
                    )
                ).onFalse(new IdleCommand(sys_elevator, sys_armPivot, sys_endEffector));

        primaryController.povLeft()
            .and(() -> !isTelopAuto)
            .whileTrue(
                DriveCommands.alignToPoint(
                    sys_drive, 
                    () -> AlignHelper.getClosestStation(sys_drive.getBlueSidePose())
                ).beforeStarting(() -> AlignHelper.reset(sys_drive.getFieldRelativeSpeeds()))
            );

        // SECONDARY CONTROLLER

        secondaryController.a()
            .onTrue(prepLevelCommand(ScoringLevel.LEVEL1));
        secondaryController.b()
            .onTrue(prepLevelCommand(ScoringLevel.LEVEL2));
        secondaryController.x()
            .onTrue(prepLevelCommand(ScoringLevel.LEVEL3));
        secondaryController.y()
            .onTrue(prepLevelCommand(ScoringLevel.LEVEL4));

        secondaryController.leftBumper()
            .onTrue(Commands.runOnce(() -> AutoCommands.scoreRight.setBoolean(false)).ignoringDisable(true));

        secondaryController.rightBumper()
            .onTrue(Commands.runOnce(() -> AutoCommands.scoreRight.setBoolean(true )).ignoringDisable(true));

        secondaryController.start()
            .onTrue(Commands.runOnce(() -> removeAlgae = true).ignoringDisable(true))
            .onFalse(Commands.runOnce(() -> removeAlgae = false).ignoringDisable(true));

        // Wanted to try something new. Could have just made a method
        BiFunction<kReefPosition, Boolean, Command> selectGenerator = new BiFunction<AutoCommands.kReefPosition,Boolean,Command>() {
            @Override
            public Command apply(kReefPosition reefPosition, Boolean scoreRight) {
                return Commands.runOnce(() -> {
                    AutoCommands.target = reefPosition;
                    AutoCommands.scoreRight.setBoolean(scoreRight);
                }).ignoringDisable(true);
            }
        };

        // CONTROL BOARD

        // SELECTION
        controlBoard.button(kButton.CLOSE_LEFT_LEFT)
            .onTrue(selectGenerator.apply(kReefPosition.CLOSE_LEFT, false));
        controlBoard.button(kButton.CLOSE_LEFT_RIGHT)
            .onTrue(selectGenerator.apply(kReefPosition.CLOSE_LEFT, true));

        controlBoard.button(kButton.CLOSE_MIDDLE_LEFT)
            .onTrue(selectGenerator.apply(kReefPosition.CLOSE, false));
        controlBoard.button(kButton.CLOSE_MIDDLE_RIGHT)
            .onTrue(selectGenerator.apply(kReefPosition.CLOSE, true));

        controlBoard.button(kButton.CLOSE_RIGHT_LEFT)
            .onTrue(selectGenerator.apply(kReefPosition.CLOSE_RIGHT, false));
        controlBoard.button(kButton.CLOSE_RIGHT_RIGHT)
            .onTrue(selectGenerator.apply(kReefPosition.CLOSE_RIGHT, true));

        controlBoard.button(kButton.FAR_LEFT_LEFT)
            .onTrue(selectGenerator.apply(kReefPosition.FAR_LEFT, false));
        controlBoard.button(kButton.FAR_LEFT_RIGHT)
            .onTrue(selectGenerator.apply(kReefPosition.FAR_LEFT, true));

        controlBoard.button(kButton.FAR_MIDDLE_LEFT)
            .onTrue(selectGenerator.apply(kReefPosition.FAR, false));
        controlBoard.button(kButton.FAR_MIDDLE_RIGHT)
            .onTrue(selectGenerator.apply(kReefPosition.FAR, true));

        controlBoard.button(kButton.FAR_RIGHT_LEFT)
            .onTrue(selectGenerator.apply(kReefPosition.FAR_RIGHT, false));
        controlBoard.button(kButton.FAR_RIGHT_RIGHT)
            .onTrue(selectGenerator.apply(kReefPosition.FAR_RIGHT, true));

        // LEVELS
        controlBoard.button(kButton.LEVEL_1)
            .onTrue(prepLevelCommand(ScoringLevel.LEVEL1));

        controlBoard.button(kButton.LEVEL_2)
            .onTrue(prepLevelCommand(ScoringLevel.LEVEL2));

        controlBoard.button(kButton.LEVEL_3)
            .onTrue(prepLevelCommand(ScoringLevel.LEVEL3));

        controlBoard.button(kButton.LEVEL_4)
            .onTrue(prepLevelCommand(ScoringLevel.LEVEL4));

        // Test Controller
        testController.a()
            .onTrue(sys_elevator.elevatorGo(Meters.of(0.4)));
        testController.b()
        .onTrue(sys_elevator.elevatorGo(Meters.of(0.2)));
        testController.x()
            .onTrue(sys_armPivot.moveArm(Degrees.of(75.0)));
        testController.y()
            .onTrue(sys_armPivot.moveArm(Degrees.of(45.0)));

    }

    public Command prepLevelCommand(ScoringLevel level) {
        return Commands.runOnce(() -> {
            Logger.recordOutput("Scoring Level", level);
            selectedScoringLevel = level;
        }).ignoringDisable(true);
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return AutoTimer.start()
            .alongWith(autoChooser.get())
            .andThen(
                Commands.parallel(
                    AutoTimer.end(kAuto.PRINT_AUTO_TIME).ignoringDisable(true),
                    Commands.either(
                        AutoCommands.telopAutoCommand(sys_drive, sys_elevator, sys_armPivot, sys_endEffector, getLevelSelectorCommand(true), () -> removeAlgae, () -> false), 
                        new IdleCommand(sys_elevator, sys_armPivot, sys_endEffector),
                        runTelop::get
                    )
                )
            );
    }
}
