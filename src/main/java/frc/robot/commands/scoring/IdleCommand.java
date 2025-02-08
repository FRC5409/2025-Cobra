package frc.robot.commands.scoring;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.kArmPivot;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.arm.ArmPivot;
import frc.robot.util.WaitThen;

public class IdleCommand extends ParallelCommandGroup {
    public IdleCommand(Elevator sys_elevator, ArmPivot sys_pivot) {
        super(
            sys_pivot.moveArm(kArmPivot.IDLE_SETPOINT),
            new WaitThen(
                () -> sys_pivot.getPosition().lte(Degrees.of(15)),
                sys_elevator.elevatorGo(Meters.of(0.05))
            )
        );
    }
}
