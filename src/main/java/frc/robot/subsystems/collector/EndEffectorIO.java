package frc.robot.subsystems.collector;

import org.littletonrobotics.junction.AutoLog;

public interface EndEffectorIO {

    @AutoLog
    public class EndEffectorInputs{
        public boolean EndEffectorConnection;
        public double EndEffectorVolts;
        public double EndEffectorCurrent;
        public double EndEffectTemp;
    }

    public default void setVoltage(double volts) {}
    public default void updateInputs(EndEffectorInputs inputs) {}
} 
