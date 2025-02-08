package frc.robot.subsystems.collector;

import org.littletonrobotics.junction.AutoLog;

public interface EndEffectorIO {

    @AutoLog
    public class EndEffectorInputs{
        public Boolean EndEffectorConnection;
        public Double EndEffectorVolts;
        public Double EndEffectorCurrent;
        public Double EndEffectTemp;
    }

    public default void setVoltage(double volts) {}
    public default void updateInputs(EndEffectorInputs inputs) {}
} 
