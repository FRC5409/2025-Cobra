package frc.robot.util.ChargedAlign;

public class PIDConstants {
    private final double p;
    private final double i;
    private final double d;

    private final double iZone;

    public PIDConstants(double p, double i, double d) {
        this(p, i, d, 1.0);
    }

    public PIDConstants(double p, double i, double d, double iZone) {
        this.p = p;
        this.i = i;
        this.d = d;

        this.iZone = iZone;
    }

    public double getP() {
        return p;
    }

    public double getI() {
        return i;
    }

    public double getD() {
        return d;
    }

    public double getIZone() {
        return iZone;
    }
}
