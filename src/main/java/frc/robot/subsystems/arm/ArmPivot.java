package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmPivot extends SubsystemBase {
    private ArmPivotIO io;
    private ArmPivotInputsAutoLogged inputs;
    Pose3d poseA = new Pose3d();
    Pose3d poseB = new Pose3d();

    StructPublisher<Pose3d> publisher = NetworkTableInstance.getDefault()
    .getStructTopic("MyPose", Pose3d.struct).publish();
    StructArrayPublisher<Pose3d> arrayPublisher = NetworkTableInstance.getDefault()
    .getStructArrayTopic("MyPoseArray", Pose3d.struct).publish();

    public ArmPivot(ArmPivotIO io) {
        this.io = io;
        inputs = new ArmPivotInputsAutoLogged();
    }

    public Command moveArm(Angle positionRad) {
        return Commands.runOnce(() -> io.moveArm(positionRad), this);
    }

    
    // public Command increaseAngleSim() {
    //     return Commands.runOnce(() -> io.increaseAngleSim(), this);oooo
    // }

    // public Command decreaseAngleSim() {
    //     return Commands.runOnce(() -> io.decreaseAngleSim(), this);
    // }

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
        publisher.set(poseA);
        arrayPublisher.set(new Pose3d[] {poseA, poseB});   
        Logger.processInputs("Arm", inputs);
    }
}