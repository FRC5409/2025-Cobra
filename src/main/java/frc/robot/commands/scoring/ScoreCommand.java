package frc.robot.commands.scoring;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ScoringLevel;
import frc.robot.Constants.kArmPivot;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.arm.ArmPivot;
import frc.robot.subsystems.collector.EndEffector;
import frc.robot.util.WaitThen;

public class ScoreCommand extends SequentialCommandGroup {
    public ScoreCommand(Elevator sys_elevator, ArmPivot sys_pivot, EndEffector sys_score, ScoringLevel level, boolean ends) {
        super(
            sys_pivot.moveArm(kArmPivot.MOVEMENT_SETPOINT),
            Commands.parallel(
                sys_elevator.elevatorGo(level.elevatorSetpoint),
                new ConditionalCommand(
                    new WaitThen(
                        () -> sys_elevator.getPosition().gte(Meters.of(0.60)),
                        sys_pivot.moveArm(level.pivotAngle)
                    ),
                    sys_pivot.moveArm(level.pivotAngle),
                    () -> level.elevatorSetpoint.gt(ScoringLevel.LEVEL3.elevatorSetpoint)
                ),
                Commands.waitUntil(() -> ends)
            )
        );
    }
}
