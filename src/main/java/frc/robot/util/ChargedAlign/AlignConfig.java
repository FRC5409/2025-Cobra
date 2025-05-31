package frc.robot.util.ChargedAlign;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.util.ErrorMessages.requireNonNullParam;

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

    private final AngularVelocity maxAngularVelo;
    private final AngularAcceleration maxAngularAccel;

    private final LinearVelocity endVelocity;

    public AlignConfig(LinearVelocity maxVelocity, LinearAcceleration maxAcceleration, Distance translationTolerance, Angle rotationTolerance, AngularVelocity maxAngularVelocity, AngularAcceleration maxAngularAcceleration, LinearVelocity endVelocity) {
        this.maxVelo  = requireNonNullParam(maxVelocity, "maxVelocity", "AlignConfig");
        this.maxAccel = requireNonNullParam(maxAcceleration, "maxAcceleration", "AlignConfig");

        this.translationTolerance = requireNonNullParam(translationTolerance, "translationTolerance", "AlignConfig");
        this.rotationTolerance    = requireNonNullParam(rotationTolerance, "translationTolerance", "AlignConfig");

        this.maxAngularVelo  = requireNonNullParam(maxAngularVelocity, "maxAngularVelocity", "AlignConfig");
        this.maxAngularAccel = requireNonNullParam(maxAngularAcceleration, "maxAngularAcceleration", "AlignConfig");

        this.endVelocity = endVelocity;

        name = "AlignConfig#" + hashCode();
    }

    public AlignConfig(LinearVelocity maxVelocity, LinearAcceleration maxAcceleration, Distance translationTolerance, Angle rotationTolerance, LinearVelocity endVelocity) {
        this(maxVelocity, maxAcceleration, translationTolerance, rotationTolerance, DegreesPerSecond.of(540), DegreesPerSecondPerSecond.of(720), endVelocity);
    }

    public AlignConfig(LinearVelocity maxVelocity, LinearAcceleration maxAcceleration, Distance translationTolerance, Angle rotationTolerance) {
        this(maxVelocity, maxAcceleration, translationTolerance, rotationTolerance, DegreesPerSecond.of(540), DegreesPerSecondPerSecond.of(720), MetersPerSecond.of(0.0));
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

    public AngularVelocity getMaxAngularVelocity() {
        return maxAngularVelo;
    }

    public AngularAcceleration getMaxAngularAcceleration() {
        return maxAngularAccel;
    }

    public LinearVelocity getEndVelocity() {
        return endVelocity;
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

    public double getEndVelocityMetersPerSecond() {
        return endVelocity.in(MetersPerSecond);
    }

    public AlignConfig withName(String name) {
        this.name = name;

        return this;
    }

    public String getName() {
        return name;
    }

    public AlignConfig copy() {
        return new AlignConfig(maxVelo, maxAccel, translationTolerance, rotationTolerance, maxAngularVelo, maxAngularAccel, endVelocity).withName(name);
    }
}
