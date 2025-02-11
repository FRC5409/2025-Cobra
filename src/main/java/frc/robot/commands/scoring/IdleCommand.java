package frc.robot.commands.scoring;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.kArmPivot;
import frc.robot.Constants.kElevator;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.arm.ArmPivot;
import frc.robot.util.WaitThen;

public class IdleCommand extends SequentialCommandGroup {
    public IdleCommand(Elevator sys_elevator, ArmPivot sys_pivot) {
        super(
            Commands.parallel(
                sys_pivot.moveArm(kArmPivot.MOVEMENT_SETPOINT),
                new WaitThen(
                    () -> sys_pivot.getPosition().gte(Degrees.of(75)),
                    sys_elevator.elevatorGo(kElevator.IDLING_HEIGHT)
                )
            ),
            sys_pivot.moveArm(kArmPivot.PICKUP_ANGLE)
        );
    }
}
