package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.Degrees;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Angle;

public interface ArmPivotIO {
    
    @AutoLog
    public class ArmPivotInputs {
        public boolean connected = false;
        public double current = 0.0;
        public double voltage = 0.0;
        public double positionAngles = 0.0;
        public double speed = 0.0;
        public double temperature = 0.0;
        public double targetAngle = 0.0;
        public double positionRad = 0.0;
    }

    public default void setVoltage(double volts) {}

    public default void updateInputs(ArmPivotInputs inputs) {}

    public default void setSetpoint(Angle armPositionRad){}

    public default Angle getPosition() {return Degrees.of(0);}

    public default void stop() {}

}