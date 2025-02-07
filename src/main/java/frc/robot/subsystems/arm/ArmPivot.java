package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Elevator.ElevatorIO;
import frc.robot.subsystems.Elevator.ElevatorIO.ElevatorInputs;
import frc.robot.util.StructHelper;

public class ArmPivot extends SubsystemBase {
    private ArmPivotIO io;
    private static ArmPivotInputsAutoLogged inputs;
    // Pose3d poseA = new Pose3d();
    // Pose3d poseB = new Pose3d();

    private Pose3d armPose;

    // StructPublisher<Pose3d> publisher = NetworkTableInstance.getDefault()
    // .getStructTopic("MyPose", Pose3d.struct).publish();
    // StructArrayPublisher<Pose3d> arrayPublisher = NetworkTableInstance.getDefault()
    // .getStructArrayTopic("MyPoseArray", Pose3d.struct).publish();

    public ArmPivot(ArmPivotIO io) {
        this.io = io;
        inputs = new ArmPivotInputsAutoLogged();

        armPose = new Pose3d();
        StructHelper.publishStruct("Arm", Pose3d.struct, () -> this.armPose);
    }

    public Command moveArm(Angle positionRad) {
        return Commands.runOnce(() -> io.moveArm(positionRad), this);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        // publisher.set(poseA);
        // arrayPublisher.set(new Pose3d[] {poseA, poseB});   
        Logger.processInputs("Arm", inputs);

        armPose = new Pose3d(0,0, Elevator.getElevatorStage2Pose3dPose(), new Rotation3d());
    }
}