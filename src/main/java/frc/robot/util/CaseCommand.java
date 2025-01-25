package frc.robot.util;

import java.util.Arrays;
import java.util.function.BooleanSupplier;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;

public class CaseCommand {
    private CaseCommand() {}

    public static final Command build(BooleanSupplier[] conditonals, Command[] commands, Command otherwise) {
        if (conditonals.length != commands.length) throw new IllegalArgumentException("Case Command recieved a different length for conditionals and commands");

        if (conditonals.length == 0) return otherwise;

        return new ConditionalCommand(
            commands[0],
            build(
                Arrays.copyOfRange(
                    conditonals,
                    1, conditonals.length
                ),
                Arrays.copyOfRange(
                    commands,
                    1, commands.length
                ),
                otherwise
            ),
            conditonals[0]
        );
    }
}
