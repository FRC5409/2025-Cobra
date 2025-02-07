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

    private static Pose3d elevatorPose;
    private static Pose3d elevatorPoseStage2;

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
        elevatorPoseStage2 = new Pose3d();
        StructHelper.publishStruct("Elevator", Pose3d.struct, ()->this.elevatorPose);  
        StructHelper.publishStruct("Elevator Stage 2", Pose3d.struct, ()->this.elevatorPoseStage2);      
    }

    public Command startManualMove(double voltage) {
        return Commands.runOnce(() -> io.setMotorVoltage(voltage), this);
    }

    public Command zeroEncoder() {
        return Commands.runOnce(() -> io.zeroEncoder(), this);
    }

    public Command elevatorGo(double setpoint) {
        return Commands.sequence(
            Commands.runOnce(() -> io.setSetpoint(setpoint), this),
            Commands.waitUntil(() -> Math.abs(setpoint - getPosition()) <= 0.05),
            Commands.runOnce(() -> io.setMotorVoltage(0.0), this)
        );
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
        elevatorPoseStage2 = new Pose3d(0,0,2*inputs.mainMotorPosition, new Rotation3d());
    }

    public static Pose3d getElevatorStage2Pose3dPose() {
        return elevatorPoseStage2;
    }
}
