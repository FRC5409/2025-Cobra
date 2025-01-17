package frc.robot.subsystems.collector;

import org.littletonrobotics.junction.AutoLog;

public interface EndEffectorIO {

    @AutoLog
    public class EndEffectorInputs{
        public boolean EndEffectorConnection = false;
        public double EndEffectorVolts = 0.0;
        public double EndEffectorCurrent = 0.0;
        public double EndEffectorPosition = 0.0;
        public double EndEffectTemp = 0.0;
    }

    public default void setVoltage(double volts) {}
    public default void setPosition(double value) {}
    public default void updateInputs(EndEffectorInputs inputs) {}
} 
