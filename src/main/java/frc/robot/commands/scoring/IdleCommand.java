package frc.robot.commands.scoring;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.arm.ArmPivot;

public class IdleCommand extends SequentialCommandGroup {
    public IdleCommand(Elevator sys_elevator, ArmPivot sys_pivot) {
        super(sys_pivot.moveArm(Degrees.of(0.0)));
    }
}
