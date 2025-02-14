package frc.robot.subsystems.collector;

import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Meters;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Distance;
public interface EndEffectorIO {
    @AutoLog
    public class EndEffectorInputs{
        public boolean endEffectorConnection = false;
        public double endEffectorVolts = 0.0;
        public double endEffectorCurrent = 0.0;
        public double endEffectTemp = 0.0;
        public Distance tofDistance = Meters.of(-1.0);
    }
    public default void setVoltage(double volts) {}
    public default void updateInputs(EndEffectorInputs inputs) {}
    public default Distance getTofRange(){ return Meter.of(0);}
} 
