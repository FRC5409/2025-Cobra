package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.util.StructHelper;

public class ArmPivot extends SubsystemBase {
    private ArmPivotIO io;
    private static ArmPivotInputsAutoLogged inputs;
  
    public Pose3d endEffectorPose;

    public ArmPivot(ArmPivotIO io) {
        this.io = io;
        inputs = new ArmPivotInputsAutoLogged();

        endEffectorPose = new Pose3d();
        StructHelper.publishStruct("End Effector location", Pose3d.struct, () -> this.endEffectorPose);
    }

    // public Command moveArm(Angle positionRad) {
    //     return Commands.runOnce(() -> io.moveArm(positionRad), this);
    // }

    public Angle getPosition() {
        return io.getPosition();
    }

    public Command moveArm(Angle positionRad) {
        return Commands.runOnce(() -> io.setSetpoint(positionRad), this);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Arm", inputs);
        endEffectorPose = new Pose3d(new Translation3d(0.22, -0.055, 0.143 + Elevator.getElevatorStage2Pose3dPose().getZ()), new Rotation3d(0, -inputs.positionRad, 0));
    }
}