package frc.robot.subsystems.Elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;

public class ElevatorIOSim implements ElevatorIO {

    private static final ShuffleboardTab sb_Sim = Shuffleboard.getTab("Sim");

    private boolean running;
    private ElevatorSim elevatorSim;
    private PIDController PID;

    private Mechanism2d mech;
    private MechanismRoot2d mainMotor;
    private MechanismRoot2d followerMotor;
    private MechanismLigament2d mainMotorMech;
    private MechanismLigament2d followerMotorMech;
    // private MechanismLigament2d endEffectorMech;

    public ElevatorIOSim() {
        elevatorSim = new ElevatorSim(DCMotor.getFalcon500Foc(1), 9.0 /1.0, 3.446, 0.0199, 0.836, 1.968, true, 1.066);

        PID = new PIDController(1.0, 0.0, 0.0);
        mech = new Mechanism2d(0.927, 10);
        mainMotor = mech.getRoot("mainMotor", 0.5, 0.25);
        followerMotor = mech.getRoot("followerMotor", 0.5, -0.25);

        mainMotorMech = mainMotor.append(new MechanismLigament2d("elevator", 1.066, 90));
        followerMotorMech = followerMotor.append(new MechanismLigament2d("elevator", 1.066, 0));

        sb_Sim.add("Elevator Mech", mech);

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
    public void updateInputs(ElevatorInputs inputs) {
        double volts = 0.0;
        double current = 0.0;
        if (running) {
            volts = MathUtil.clamp(PID.calculate(elevatorSim.getPositionMeters()) * 12, -RoboRioSim.getVInVoltage(), RoboRioSim.getVInVoltage());
            current = elevatorSim.getCurrentDrawAmps();
        }
        elevatorSim.setInputVoltage (volts);
        elevatorSim.update(0.02);
        mainMotorMech.setLength((elevatorSim.getPositionMeters()));
        followerMotorMech.setLength(elevatorSim.getPositionMeters());

        inputs.mainMotorConnected = true;
        inputs.mainAppliedVoltage = volts;
        inputs.mainAppliedCurrent = current;
        inputs.mainMotorTemperature = 0.0;
        inputs.mainMotorPosition = mainMotorMech.getLength();

        inputs.followerMotorConnected = true;
        inputs.followerAppliedVoltage = volts;
        inputs.followerAppliedCurrent = current;
        inputs.followerMotorTemperature = 0.0;
        inputs.followerMotorPosition = followerMotorMech.getLength();

    }
}