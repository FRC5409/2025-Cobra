package frc.robot.util.ChargedAlign;

import java.util.function.BooleanSupplier;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class RunUntilCommand extends FunctionalCommand {
    public RunUntilCommand(BooleanSupplier runUntil, Subsystem... requirements) {
        super(() -> {}, () -> {}, interupted -> {}, runUntil, requirements);
    }
}
