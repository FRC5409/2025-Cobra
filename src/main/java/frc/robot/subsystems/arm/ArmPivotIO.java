package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.wpilibj2.command.Command;

public interface ArmPivotIO {
    
    @AutoLog
    public class ArmPivotInputs {
        public boolean armConnected = false;
        public double armCurrent = 0.0;
        public double armVolts = 0.0;
        public double armPosition = 0.0;
        public double armSpeed = 0.0;
        public double armTemp = 0.0;
        public double armPositionRad = 0.0;
    } 

    public default void setVoltage(double volts) {}

    public default void updateInputs(ArmPivotInputs inputs) {}

    // public default void setPosition(double position) {}

    public default void setPositionUp(double position) {}

    public default void setPositionDown(double position) {}

    // public default void getConversionFactor() {}

    public default void keepPosition(double armPositionRad) {}
    
}