package frc.robot.util.ChargedAlign;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.util.ErrorMessages.requireNonNullParam;

import java.util.Optional;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;

public class AlignConfig {

    private String name;

    private final LinearVelocity maxVelo;
    private final LinearAcceleration maxAccel;

    private final Distance translationTolerance;
    private final Angle rotationTolerance;

    private final Optional<AngularVelocity> maxAngularVelo;
    private final Optional<AngularAcceleration> maxAngularAccel;

    public AlignConfig(LinearVelocity maxVelocity, LinearAcceleration maxAcceleration, Distance translationTolerance, Angle rotationTolerance, AngularVelocity maxAngularVelocity, AngularAcceleration maxAngularAcceleration) {
        this.maxVelo  = requireNonNullParam(maxVelocity, "maxVelocity", "AlignConfig");
        this.maxAccel = requireNonNullParam(maxAcceleration, "maxAcceleration", "AlignConfig");

        this.translationTolerance = requireNonNullParam(translationTolerance, "translationTolerance", "AlignConfig");
        this.rotationTolerance = requireNonNullParam(rotationTolerance, "translationTolerance", "AlignConfig");

        this.maxAngularVelo  = Optional.ofNullable(maxAngularVelocity);
        this.maxAngularAccel = Optional.ofNullable(maxAngularAcceleration);

        name = "AlignConfig#" + hashCode();
    }

    public AlignConfig(LinearVelocity maxVelocity, LinearAcceleration maxAcceleration, Distance translationTolerance, Angle rotationTolerance) {
        this(maxVelocity, maxAcceleration, translationTolerance, rotationTolerance, null, null);
    }

    public AlignConfig() {
        this(MetersPerSecond.of(3.0), MetersPerSecondPerSecond.of(4.0), Centimeters.of(3.0), Degrees.of(1.0));
    }

    public LinearVelocity getMaxVelocity() {
        return maxVelo;
    }

    public LinearAcceleration getMaxAcceleration() {
        return maxAccel;
    }

    public Distance getTranslationTolerance() {
        return translationTolerance;
    }

    public Angle getRotationTolerance() {
        return rotationTolerance;
    }

    public Optional<AngularVelocity> getMaxAngularVelocity() {
        return maxAngularVelo;
    }

    public Optional<AngularAcceleration> getMaxAngularAcceleration() {
        return maxAngularAccel;
    }

    public double getMaxVelocityMetersPerSecond() {
        return maxVelo.in(MetersPerSecond);
    }

    public double getMaxAccelerationMetersPerSecondPerSecond() {
        return maxAccel.in(MetersPerSecondPerSecond);
    }

    public double getTranslationToleranceMeters() {
        return translationTolerance.in(Meters);
    }

    public double getRotationToleranceRadians() {
        return rotationTolerance.in(Radians);
    }

    public AlignConfig withName(String name) {
        this.name = name;

        return this;
    }

    public String getName() {
        return name;
    }
}
