package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import frc.robot.Constants.kElevator;

public class ElevatorIOSim implements ElevatorIO {

    private static final ShuffleboardTab sb_sim = Shuffleboard.getTab("SIM");

    private boolean isRunning;

    // Physics Simulation
    private final ElevatorSim elevatorSim;
    private final PIDController controller;

    // Visualization
    private final Mechanism2d mech;
    private final MechanismRoot2d root;
    private final MechanismLigament2d ligament;

    public ElevatorIOSim() {
        elevatorSim = new ElevatorSim(
            DCMotor.getFalcon500Foc(1), // Number of different types of motors. Change to Non-FOC if not using FOC control
            kElevator.ELEVATOR_GEARING, 
            kElevator.ELEVATOR_MASS.in(Kilograms),
            kElevator.ELEVATOR_DRUM_RADIUS.in(Meters),
            0.5, // The height of the elevator when it is stored
            0.8, // The height of the elevator when it is fully extended
            true,
            0.5 // Where the elevator will start
        );

        controller = new PIDController(
            kElevator.SIMULATED_PID_VALUES.kP,
            kElevator.SIMULATED_PID_VALUES.kI,
            kElevator.SIMULATED_PID_VALUES.kD
        );

        // Width should be double the width of the physical mechanism.
        // Don't worry about height
        mech = new Mechanism2d(0.6, 10.0); 
        root = mech.getRoot("Elevator", 0.3, 0.1); // Forwards/backwards, Up/down

        ligament = root.append(
            new MechanismLigament2d(
                "Structure",
                0.5, // Starting height
                90 // Facing UP
            )
        );

        sb_sim.add("Elevator Mech", mech);

        isRunning = false;
    }

    @Override
    public void setPosition(Distance distance) {
        controller.setSetpoint(distance.in(Meters));
        isRunning = true;
    }

    @Override
    public void stop() {
        elevatorSim.setInputVoltage(0.0);
        controller.reset();
        isRunning = false;
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        double voltage = 0.0;
        double current = 0.0;
        if (isRunning) {
            // Calculates the output voltage from the PID controller
            voltage = MathUtil.clamp(
                controller.calculate(elevatorSim.getPositionMeters()) * RoboRioSim.getVInVoltage(),
                -12,
                12
            );
            elevatorSim.setInputVoltage(voltage);
            elevatorSim.update(0.02); // 50 ticks per second (1 / 50 = 0.02)

            ligament.setLength(elevatorSim.getPositionMeters());

            current = elevatorSim.getCurrentDrawAmps();
        }

        inputs.motorConnect = true;
        inputs.positionMeters = ligament.getLength();
        inputs.targetMeters = controller.getSetpoint();
        inputs.voltage = voltage;
        inputs.current = current;
        inputs.tempature = 0.0;
    }
}
