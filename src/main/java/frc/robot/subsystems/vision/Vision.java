package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.Logger;
import org.opencv.core.Point;

// 5409: The Chargers
// http://github.com/FRC5409

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
    private final VisionIO io;
    private final VisionInputsAutoLogged inputs;

    public Vision(VisionIO io) {
        this.io = io;
        inputs = new VisionInputsAutoLogged();
    }

    public Point get2DOffset() {
        return new Point(inputs.tx, inputs.ty);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        io.updateInputs(inputs);
        Logger.processInputs("Vision", inputs);
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}