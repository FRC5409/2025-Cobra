package frc.robot.util.ChargedAlign;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.units.measure.Time;

/**
 * A fixed version of {@link ProfiledPIDController} to suit our needs.
 * We found there were issues with reseting the velocity of the robot in wpilibs Profiled Cotroller so this is a simplified controller that does pretty much the same
 * @author Alexander Szura
 */
public class ProfiledController {

    private final PIDController controller;
    private double maxVelocity;
    private double maxAcceleration;

    private double lastPoint;

    private boolean continuous = false;
    private double minimumInput;
    private double maximumInput;


    /**
     * New ProfiledController class
     * @param PID The PID constants of the system
     * @param maxVelocity the max velocity of the system in a unit of your choice
     * @param maxAcceleration The max accleration of the system in a unit of your choice
     * @param period How often the PID controller is updated
     */
    public ProfiledController(PIDConstants PID, double maxVelocity, double maxAcceleration, Time period) {
        controller = new PIDController(PID.getP(), PID.getI(), PID.getD(), period.in(Seconds));
        controller.setIZone(PID.getIZone());

        setContraints(maxVelocity, maxAcceleration);
    }
    

    /**
     * New ProfiledController class with a period of 0.02 seconds or 50hz
     * @param PID The PID constants of the system
     * @param maxVelocity the max velocity of the system in a unit of your choice
     * @param maxAcceleration The max accleration of the system in a unit of your choice
     */
    public ProfiledController(PIDConstants PID, double maxVelocity, double maxAcceleration) {
        this(PID, maxVelocity, maxAcceleration, Seconds.of(0.02));
    }

    /**
     * New ProfiledController class with a period of 0.02 seconds or 50hz
     * @param kP P value
     * @param kI I value
     * @param kD D valuee
     * @param maxVelocity the max velocity of the system in a unit of your choice
     * @param maxAcceleration The max accleration of the system in a unit of your choice
     */
    public ProfiledController(double kP, double kI, double kD, double maxVelocity, double maxAcceleration) {
        this(new PIDConstants(kP, kI, kD), maxVelocity, maxAcceleration);
    }

    /**
     * Updates the contraints of the system
     * @param velo new max velocity
     * @param accel new max acceleration
     */
    public void setContraints(double velo, double accel) {
        setMaxVelocity(velo);
        setMaxAcceleration(accel);
    }

    /**
     * Updates the max velocity of the system
     * @param velo new max velocity
     */
    public void setMaxVelocity(double velo) {
        maxVelocity = Math.max(velo, 0.0);
    }

    /**
     * Updates the max acceleration of the system
     * @param accel new max acceleration
     */
    public void setMaxAcceleration(double accel) {
        maxAcceleration = Math.max(accel, 0.0);
    }

    /**
     * Enables continuous input.
     * <p>Rather then using the max and min input range as constraints, it considers them to be the same point and automatically calculates the shortest route to the setpoint.
     * @param minimumInput The minimum value expected from the input.
     * @param maximumInput The maximum value expected from the input.
     */
    public void enableContinuousInput(double minimumInput, double maximumInput) {
        continuous = true;
        this.minimumInput = minimumInput;
        this.maximumInput = maximumInput;
        controller.enableContinuousInput(minimumInput, maximumInput);
    }

    /**
     * Disables continuous input.
     */
    public void disableContinuousInput() {
        continuous = false;
        controller.disableContinuousInput();
    }

    /**
     * @return TThe current max velocity of the system
     */
    public double getMaxVelocity() {
        return maxVelocity;
    }

    /**
     * @return TThe current max acceleration of the system
     */
    public double getMaxAcceleration() {
        return maxAcceleration;
    }

    /**
     * Resets the controller with a velocity of 0.0
     */
    public void reset() {
        reset(0.0);
    }

    /**
     * Resets the controller
     * @param velocity the inital velocity of the system
     */
    public void reset(double velocity) {
        controller.reset();
        lastPoint = velocity;
    }

    /**
     * Calculates the next output of the system
     * @param measurement what the current position of the system
     * @param setpoint the target position
     * @return the Profiled output
     */
    public double calculate(double measurement, double setpoint) {
        double dt = controller.getPeriod();
    
        double error = setpoint - measurement;
        if (continuous) {
            double range = maximumInput - minimumInput;
            error = Math.IEEEremainder(error, range);
            setpoint = measurement + error;
        }

        double output = controller.calculate(measurement, setpoint);
    
        output = Math.max(-maxVelocity, Math.min(maxVelocity, output));
    
        double deltaVelocity = output - lastPoint;
        double constrainedVelocity = lastPoint + Math.max(-maxAcceleration * dt, Math.min(maxAcceleration * dt, deltaVelocity));
    
        lastPoint = constrainedVelocity;
        
        return constrainedVelocity;
    }
    
}
