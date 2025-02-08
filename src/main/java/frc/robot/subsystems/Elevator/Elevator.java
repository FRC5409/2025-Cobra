// 5409: The Chargers
// http://github.com/FRC5409

package frc.robot.subsystems.Elevator;

import static edu.wpi.first.units.Units.Meters;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
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

    private static Pose3d elevatorPose;
    private static Pose3d elevatorPoseStage2;

    private final Alert leftElevatorAlert  = new Alert("The Left Elevator Motor is Disconnected " + kElevator.MAIN_MOTOR_ID, AlertType.kError);
    private final Alert rightElevatorAlert = new Alert("The Right Elevator Motor is Disconnected " + kElevator.FOLLOWER_MOTOR_ID, AlertType.kError);
    public Elevator(ElevatorIO io) {
        // IO
        this.io = io;
        inputs = new ElevatorInputsAutoLogged();

        elevatorPose = new Pose3d();
        elevatorPoseStage2 = new Pose3d();
        StructHelper.publishStruct("Elevator", Pose3d.struct, ()->Elevator.elevatorPose);  
        StructHelper.publishStruct("Elevator Stage 2", Pose3d.struct, ()->Elevator.elevatorPoseStage2);      
    }

    public Command startManualMove(double voltage) {
        return Commands.runOnce(() -> io.setMotorVoltage(voltage), this);
    }

    public Command zeroEncoder() {
        return Commands.runOnce(() -> io.zeroEncoder(), this);
    }

    public Command elevatorGo(Distance setpoint) {
        return Commands.sequence(
            Commands.runOnce(() -> io.setSetpoint(setpoint.in(Meters)), this),
            Commands.waitUntil(() -> Math.abs(setpoint.in(Meters) - getPosition()) <= 0.02),
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
        Logger.processInputs("Elevator", inputs);
        leftElevatorAlert.set(!inputs.mainMotorConnection);
        rightElevatorAlert.set(!inputs.followerMotorConnection);
        elevatorPose = new Pose3d(0,0,inputs.mainMotorPosition, new Rotation3d());
        elevatorPoseStage2 = new Pose3d(0,0,2*inputs.mainMotorPosition, new Rotation3d());
    }

    public static Pose3d getElevatorStage2Pose3dPose() {
        return elevatorPoseStage2;
    }
}
