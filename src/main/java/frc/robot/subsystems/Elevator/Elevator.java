// 5409: The Chargers
// http://github.com/FRC5409

package frc.robot.subsystems.Elevator;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants.kElevator;

public class Elevator extends SubsystemBase{

    // IO
    private final ElevatorIO io;
    private final ElevatorInputsAutoLogged inputs;

    // Shuffleboard
    private final ShuffleboardTab sb_tab;

    //ghost pose3d
    private Pose3d poseA = new Pose3d();
    private Pose3d poseB = new Pose3d();
    StructPublisher<Pose3d> publisher = NetworkTableInstance.getDefault()
    .getStructTopic("MyPose", Pose3d.struct).publish();
    StructArrayPublisher<Pose3d> arrayPublisher = NetworkTableInstance.getDefault()
    .getStructArrayTopic("MyPoseArray", Pose3d.struct).publish();


    public Elevator(ElevatorIO io) {
        // IO
        this.io = io;
        inputs = new ElevatorInputsAutoLogged();

        // Shuffleboard
        sb_tab = Shuffleboard.getTab("Elevator");
        sb_tab.addDouble("Elevator Position", () -> io.getPosition());
    }

    public Command startManualMove(double voltage) {
        return Commands.runOnce(() -> io.setMotorVoltage(voltage), this);
    }

    public Command zeroEncoder() {
        return Commands.runOnce(() -> io.zeroEncoder(), this);
    }

    public Command ElevatorGo(double setpoint) {
        return Commands.runOnce(() -> io.setSetpoint(setpoint), this);
    }

    public Command stopAll() {
        return Commands.runOnce(() -> io.stopMotor(), this);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        io.updateInputs(inputs);
        publisher.set(poseA);
        arrayPublisher.set(new Pose3d[] {poseA, poseB});
        Logger.processInputs("Elevator", inputs);
    }

}
