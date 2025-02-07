package frc.robot.commands.scoring;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.RobotContainer.ScoringLevel;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.arm.ArmPivot;
import frc.robot.subsystems.collector.EndEffector;

public class ScoreCommand extends SequentialCommandGroup {
    public ScoreCommand(Elevator sys_elevator, ArmPivot sys_pivot, EndEffector sys_score, ScoringLevel level) {
        super(
            Commands.parallel(
                sys_elevator.elevatorGo(level.elevatorSetpoint),
                sys_pivot.moveArm(level.pivotAngle)
            ),
            sys_score.runUntilCoralNotDetected(3).unless(() -> Constants.currentMode == Mode.SIM)
        );
    }
}
