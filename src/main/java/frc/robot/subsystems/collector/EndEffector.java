package frc.robot.subsystems.collector;

import org.littletonrobotics.junction.Logger;

import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kEndEffector;

public class EndEffector extends SubsystemBase {

    private final EndEffectorIO io;
    private final EndEffectorInputsAutoLogged inputs = new EndEffectorInputsAutoLogged();
    private final TimeOfFlight tof = new TimeOfFlight(kEndEffector.TIMOFFLIGHT_SENSORID);
    private boolean coralDetected = false;

    public EndEffector(EndEffectorIO io) {
        this.io = io;
        tof.setRangingMode(RangingMode.Short, 50);
    }

    public Command runUntilCoralDetected(double voltage) {
        return Commands.parallel(
                Commands.runOnce(() -> io.setVoltage(voltage), this),
                Commands.waitUntil(
                    () -> coralDetected
            )
        ).andThen(
            Commands.waitSeconds(0.5),
            Commands.runOnce(()-> io.setVoltage(0),this)
            );

    }

    public Command setVoltage(double voltage){
        return Commands.runOnce(() -> io.setVoltage(voltage), this);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        io.updateInputs(inputs);
        Logger.processInputs("EndEffector", inputs);
        if (tof.getRange() <= kEndEffector.TIMEOFFLIGHT_DISTANCE_VALIDATION){
            coralDetected = true;
        } else {
            coralDetected = false;
        }
        // System.out.println(tof.getRange());  --- Use when testing
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }

}