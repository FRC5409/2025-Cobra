package frc.robot.subsystems.collector;

import org.littletonrobotics.junction.AutoLog;

public interface EndEffectorIO {

    @AutoLog
    public class EndEffectorInputs{
        public boolean endEffectorConnection;
        public double endEffectorVolts;
        public double endEffectorCurrent;
        public double endEffectTemp;
    }

    public default void setVoltage(double volts) {}
    public default void updateInputs(EndEffectorInputs inputs) {}
} 
