// 5409: The Chargers
// http://github.com/FRC5409

package frc.robot.subsystems.Elevator;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants.kElevator;

/**
 * @author BN
 */
public class Elevator extends SubsystemBase{
    private static Elevator instance = null;

    // IO
    private final ElevatorIO io;
    private final ElevatorInputsAutoLogged inputs;

    // Shuffleboard
    private final ShuffleboardTab sb_tab;

    private Elevator(ElevatorIO io) {
        // IO
        this.io = io;
        inputs = new ElevatorInputsAutoLogged();

        // Shuffleboard
        sb_tab = Shuffleboard.getTab("Elevator");
        sb_tab.addDouble("Elevator Position", () -> io.getPosition());
    }

    /**
     * Get subsystem
     * @param io
     * @return This
     */
    public static Elevator createInstance(ElevatorIO io) {
        if (instance != null) throw new RuntimeException("Elevator has already been created!");

        return instance = new Elevator(io);
    }

    /**
     * Manually begin moving
     * @param voltage
     * @return the command
     */
    public Command startManualMove(double voltage) {
        return Commands.runOnce(() -> io.setMotorVoltage(voltage), this);
    }

    public Command zeroEncoder() {
        return Commands.runOnce(() -> io.zeroEncoder(), this);
    }

    public Command ElevatorGo(double setpoint) {
        return Commands.runOnce(() -> io.setSetpoint(setpoint), this);
    }

    /**
     * Stop all motors
     * @return The command
     */
    public Command stopAll() {
        return Commands.runOnce(() -> io.stopMotor(), this);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        io.updateInputs(inputs);
        Logger.processInputs("Elevator", inputs);
    }

}
