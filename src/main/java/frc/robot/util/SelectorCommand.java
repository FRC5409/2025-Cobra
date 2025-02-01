package frc.robot.util;

import static edu.wpi.first.util.ErrorMessages.requireNonNullParam;

import java.util.function.BooleanSupplier;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * @author Alexander Szura
 */
public class SelectorCommand extends Command {

    private final Command onTrue;
    private final Command onFalse;
    private final BooleanSupplier condition;

    private Command currentCommand;
    private boolean lastEntry;

    public SelectorCommand(Command onTrue, Command onFalse, BooleanSupplier condition) {
        this.onTrue = requireNonNullParam(onTrue, "onTrue", "SelectorCommand");
        this.onFalse = requireNonNullParam(onFalse, "onTrue", "SelectorCommand");
        this.condition = requireNonNullParam(condition, "onTrue", "SelectorCommand");

        CommandScheduler.getInstance().registerComposedCommands(onTrue, onFalse);
        
        addRequirements(onTrue.getRequirements());
        addRequirements(onFalse.getRequirements());
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        lastEntry = condition.getAsBoolean();
        if (lastEntry)
            currentCommand = onTrue;
        else
            currentCommand = onFalse;

        currentCommand.initialize();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        boolean currentEntry = condition.getAsBoolean();
        if (lastEntry != currentEntry) {
            currentCommand.end(true);
            initialize();
        }

        currentCommand.execute();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        currentCommand.end(interrupted);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return currentCommand.isFinished();
    }

}
