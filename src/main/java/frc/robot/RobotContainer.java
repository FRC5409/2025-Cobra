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

import java.util.Map;
import java.io.IOException;
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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.Mode;
import frc.robot.Constants.kAuto;
import frc.robot.Constants.kDrive;
import frc.robot.commands.AutoCommands;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.AutoCommands.kReefPosition;
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
import frc.robot.commands.scoring.L1Scoring;
import frc.robot.commands.scoring.L2Scoring;
import frc.robot.commands.scoring.L3Scoring;
import frc.robot.commands.scoring.L4Scoring;
import frc.robot.commands.scoring.SubPickup;
import frc.robot.util.AutoTimer;
import frc.robot.util.DebugCommand;
import frc.robot.util.OpponentRobot;
import frc.robot.util.WaitThen;
import frc.robot.util.AlignHelper.kClosestType;
import frc.robot.util.AlignHelper.kDirection;

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
    private enum ScoringLevel {
      LEVEL1, LEVEL2, LEVEL3, LEVEL4
    }
  
    // Subsystems
    protected final Drive       sys_drive;
    protected final Vision      sys_vision;
    protected final Elevator    sys_elevator;
    protected final EndEffector sys_endEffector;
    protected final ArmPivot    sys_armPivot;

    private SwerveDriveSimulation simConfig;

    // Commands
    protected final Command telopAutoCommand;
  
    private ScoringLevel selectedScoringLevel = ScoringLevel.LEVEL1;
    private Command selectScoringCommand;
    private final SubPickup seq_pickUp;

    // Controller
    private final CommandXboxController primaryController = new CommandXboxController(0);
    private final CommandXboxController secondaryController = new CommandXboxController(1);

    public static boolean isTelopAuto = false;

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
              
                sys_armPivot = new ArmPivot(new ArmPivotIOTalonFX(0,0));
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
                sys_endEffector = new EndEffector(new EndEffectorIO() {});
                
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

                final OpponentRobot sys_opponent = 
                    new OpponentRobot(new Pose2d(3, 3, Rotation2d.fromDegrees(0.0)));
                sys_opponent.setDefaultCommand(sys_opponent.joystickDrive(secondaryController));
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
        telopAutoCommand = AutoCommands.telopAutoCommand(sys_drive, () -> primaryController.getHID().getPOV() != -1).alongWith(
                Commands.run(() -> {
                    if (Math.hypot(primaryController.getRightX(), primaryController.getRightY()) < 0.1) return;

                    Angle targetAngle = Radians.of(Math.atan2(primaryController.getRightY(), primaryController.getRightX()));

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

                    // AutoCommands.target = 
                })
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
                                                0.5,
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
                                                0.5,
                                                Commands.runOnce(
                                                        () -> primaryController
                                                                .setRumble(RumbleType.kBothRumble,
                                                                        0.0)))
                                                .ignoringDisable(true)))
                .onFalse(
                        Commands.runOnce(() -> secondaryDisconnectedAlert.set(false))
                                .ignoringDisable(true));

        // TODO: fix on real robot
        new Trigger(DriverStation::isDSAttached).onTrue(
                Commands.runOnce(() -> {
                    primaryDisconnectedAlert.set(!primaryController.isConnected());
                    secondaryDisconnectedAlert.set(!secondaryController.isConnected());

                    resetPose();
                }).ignoringDisable(true));
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
        NamedCommands.registerCommand(
                "ALIGN_LEFT",
                new ConditionalCommand(
                    DriveCommands.alignToPoint(sys_drive,
                        () -> AlignHelper.getClosestReef(
                            sys_drive.getBlueSidePose(),
                            kClosestType.DISTANCE,
                            kDirection.LEFT
                        )
                    ),
                    DriveCommands.alignToPoint(sys_drive,
                        () -> AlignHelper.getClosestReef(
                            sys_drive.getBlueSidePose(),
                            kClosestType.DISTANCE,
                            kDirection.RIGHT
                        )
                    ),
                    () -> !autoChooser.getSendableChooser().getSelected().startsWith("{R}")
                )
            );

        NamedCommands.registerCommand(
                "ALIGN_RIGHT",
                new ConditionalCommand(
                    DriveCommands.alignToPoint(sys_drive,
                        () -> AlignHelper.getClosestReef(
                            sys_drive.getBlueSidePose(),
                            kClosestType.DISTANCE,
                            kDirection.RIGHT
                        )
                    ),
                    DriveCommands.alignToPoint(sys_drive,
                        () -> AlignHelper.getClosestReef(
                            sys_drive.getBlueSidePose(),
                            kClosestType.DISTANCE,
                            kDirection.LEFT
                        )
                    ),
                    () -> !autoChooser.getSendableChooser().getSelected().startsWith("{R}")
                )
            );

        // TODO: Finish Commands
        NamedCommands.registerCommand("PREPARE_STATION", Commands.none());
        NamedCommands.registerCommand("STATION_PICKUP", Commands.waitSeconds(0.35));
        NamedCommands.registerCommand("SCORE_CORAL", Commands.waitSeconds(0.45));
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

        primaryController.x().onTrue(DriveCommands.setSpeedHigh(sys_drive));

        primaryController.y().onTrue(DriveCommands.setSpeedLow(sys_drive));

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
                .whileFalse(
                    Commands.runOnce(() -> isTelopAuto = !isTelopAuto)
                        .andThen(telopAutoCommand)
                );

        primaryController.leftBumper()
            .and(() -> isTelopAuto)
            .onTrue(Commands.runOnce(() -> AutoCommands.scoreRight.setBoolean(false)).ignoringDisable(true));

        primaryController.rightBumper()
            .and(() -> isTelopAuto)
            .onTrue(Commands.runOnce(() -> AutoCommands.scoreRight.setBoolean(true )).ignoringDisable(true));

        primaryController.rightBumper()
            .and(() -> !isTelopAuto)
            .onTrue( Commands.runOnce(() -> sys_drive.coastMode()).ignoringDisable(true))
            .onFalse(Commands.runOnce(() -> sys_drive.brakeMode()).ignoringDisable(true));

        primaryController.y()
            .onTrue(
                DriveCommands.setSpeedLow(sys_drive) 
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
            .povLeft()
            .and(() -> !isTelopAuto)
                .whileTrue(
                    DriveCommands.alignToPoint(
                        sys_drive, 
                        () -> AlignHelper.getClosestElement(sys_drive.getBlueSidePose(), kDirection.LEFT)
                    ).beforeStarting(() -> AlignHelper.reset(sys_drive.getFieldRelativeSpeeds()))
                );

        primaryController
            .povRight()
            .and(() -> !isTelopAuto)
                .whileTrue(
                    DriveCommands.alignToPoint(
                        sys_drive, 
                        () -> AlignHelper.getClosestElement(sys_drive.getBlueSidePose(), kDirection.RIGHT)
                    ).beforeStarting(() -> AlignHelper.reset(sys_drive.getFieldRelativeSpeeds()))
                );
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
                AutoTimer.end(kAuto.PRINT_AUTO_TIME).ignoringDisable(true)
                .alongWith(
                    AutoCommands.telopAutoCommand(sys_drive, () -> false).onlyIf(runTelop::get)
                )
            );
    }
}
