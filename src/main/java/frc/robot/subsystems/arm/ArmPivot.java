package frc.robot.subsystems.arm;

import java.util.logging.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmPivot extends SubsystemBase {
    private ArmPivotIO io;
    private ArmPivotInputsAutoLogged inputs;

    public ArmPivot(ArmPivotIO io) {
        this.io = io;
        inputs = new ArmPivotInputsAutoLogged();
    }

    public Command setPositionUp(){
        return Commands.runOnce(() -> io.setPositionUp(10), this);
    }

    public Command setPositionDown(){
        return Commands.runOnce(() -> io.setPositionDown(-10), this);
    }

    public Command keepPosition(double armPositionRad) {
        return Commands.runOnce(() -> io.keepPosition(armPositionRad), this);
    }

    // public Command startExtendingArm() {
    //     return Commands.runOnce(() -> io.setVoltage(6), this);
    // }

    // public Command startRetracktingArm() {
    //     return Commands.runOnce(() -> io.setVoltage(-6), this);
    // }

    // public Command stopArm() {
    //     return Commands.runOnce(() -> io.setVoltage(0), this);
    // }
    
    @Override
    public void periodic() {
        io.updateInputs(inputs);
        // Logger.processInputs("Arm", inputs);
    }
}