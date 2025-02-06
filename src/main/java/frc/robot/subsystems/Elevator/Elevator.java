// 5409: The Chargers
// http://github.com/FRC5409

package frc.robot.subsystems.Elevator;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kElevator;
import frc.robot.util.StructHelper;

public class Elevator extends SubsystemBase{

    // IO
    private final ElevatorIO io;
    private final ElevatorInputsAutoLogged inputs;

    // Shuffleboard
    private final ShuffleboardTab sb_tab;

    private Pose3d elevatorPose;

    private final Alert leftElevatorAlert = new Alert("The Left Motor is Disconnected " + kElevator.MAIN_MOTOR_ID, AlertType.kError);
    private final Alert rightElevatorAlert = new Alert("The Rigth Motor is Disconnected " + kElevator.FOLLOWER_MOTOR_ID, AlertType.kError);
    public Elevator(ElevatorIO io) {
        // IO
        this.io = io;
        inputs = new ElevatorInputsAutoLogged();

        // Shuffleboard
        sb_tab = Shuffleboard.getTab("Elevator");
        sb_tab.addDouble("Elevator Position", () -> io.getPosition());

        elevatorPose = new Pose3d();
        StructHelper.publishStruct("Elevator", Pose3d.struct, ()->this.elevatorPose);        
    }

    public Command startManualMove(double voltage) {
        return Commands.runOnce(() -> io.setMotorVoltage(voltage), this);
    }

    public Command zeroEncoder() {
        return Commands.runOnce(() -> io.zeroEncoder(), this);
    }

    public Command ElevatorGo(double setpoint) {
        return Commands.runOnce(() -> io.setSetpoint(setpoint), this)
        .until(() -> Math.abs(setpoint - getPosition()) <= 5);
    }

    public Command stopAll() {
        return Commands.runOnce(() -> io.stopMotor(), this);
    }
    
    public double getPosition() {
        return io.getPosition();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        io.updateInputs(inputs);
        leftElevatorAlert.set(!inputs.mainMotorConnected);
        rightElevatorAlert.set(!inputs.followerMotorConnected);
        elevatorPose = new Pose3d(0,0,inputs.mainMotorPosition, new Rotation3d());
    }
}
