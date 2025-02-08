package frc.robot.subsystems.Elevator;

import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import frc.robot.Constants.kElevator;

public class ElevatorIOSim implements ElevatorIO {

    private boolean running;
    private ElevatorSim elevatorSim;
    private PIDController PID;

    public ElevatorIOSim() {
        elevatorSim = new ElevatorSim(
            DCMotor.getFalcon500Foc(2), 
            kElevator.kGearing, 
            kElevator.ELEVATOR_MASS.in(Kilograms), 
            kElevator.ELEVATOR_DRUMRADIUS.in(Meters), 
            kElevator.ELEVATOR_MIN_HEIGHT, 
            kElevator.ELEVATOR_MAX_HEIGHT, 
            true, 
            kElevator.ELEVATOR_MIN_HEIGHT
        );
        PID = new PIDController(kElevator.SIM_PID.kP, kElevator.SIM_PID.kI, kElevator.SIM_PID.kD);
        running = false;

    }

    @Override
    public void setMotorVoltage(double voltage) {
        elevatorSim.setInputVoltage(voltage);
    }

    @Override
    public void stopMotor() {
        elevatorSim.setInputVoltage(0.0);
        running = false;
    }

    @Override
    public void setSetpoint(double setpoint) {
        PID.setSetpoint(setpoint);
        running = true;
    }

    @Override
    public double getPosition() {
        return elevatorSim.getPositionMeters();
    }

    @Override
    public void updateInputs(ElevatorInputs inputs) {
        double volts = 0.0;
        double current = 0.0;
        if (running) {
            volts = MathUtil.clamp(
                PID.calculate(elevatorSim.getPositionMeters()) * 12, 
                -RoboRioSim.getVInVoltage(), 
                RoboRioSim.getVInVoltage()
            );
            current = elevatorSim.getCurrentDrawAmps();
        }
        elevatorSim.setInputVoltage (volts);
        elevatorSim.update(0.02);

        inputs.mainMotorConnection = true;
        inputs.mainAppliedVoltage = volts;
        inputs.mainAppliedCurrent = current;
        inputs.mainMotorTemperature = 0.0;
        inputs.mainMotorPosition = elevatorSim.getPositionMeters();

        inputs.followerMotorConnection = true;
        inputs.followerAppliedVoltage = volts;
        inputs.followerAppliedCurrent = current;
        inputs.followerMotorTemperature = 0.0;
        inputs.followerMotorPosition = elevatorSim.getPositionMeters();

    }
}