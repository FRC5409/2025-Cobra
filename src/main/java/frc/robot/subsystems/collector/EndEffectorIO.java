package frc.robot.subsystems.collector;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Millimeters;

import org.littletonrobotics.junction.AutoLog;
import edu.wpi.first.units.measure.Distance;
public interface EndEffectorIO {
    @AutoLog
    public class EndEffectorInputs{
        public boolean endEffectorConnection = false;
        public double endEffectorVolts = 0.0;
        public double endEffectorCurrent = 0.0;
        public double endEffectorTemp = 0.0;
        public double tofDistance = -1;
        public double endEffectorVelocity = 0.0;
        public double endEffectorPosition = 0.0;
    }
    public default void setVoltage(double volts) {}
    public default void updateInputs(EndEffectorInputs inputs) {}
    public default Distance getTofRange(){ return Millimeters.of(0);}
    public default double getMotorCurrent() { return 0.0; }
} 



// public default Distance getTofRange_L1(){return Millimeters.of(0);}

