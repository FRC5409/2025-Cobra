package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;
import edu.wpi.first.units.measure.Distance;

public interface ElevatorIO {

    @AutoLog
    public class ElevatorIOInputs {
        public boolean motorConnect = false;
        public double targetMeters = 0.0;
        public double positionMeters = 0.0;
        public double voltage = 0.0;
        public double current = 0.0;
        public double tempature = 0.0;
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(ElevatorIOInputs inputs) {}

    /**
     * Uses PID to attempt to get the elevator to the specifed distance
     * @param distance the distance to achieve
     */
    public default void setPosition(Distance distance) {}
    
    /** Stops moving the elevator */
    public default void stop() {}
}
