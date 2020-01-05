package org.firstinspires.ftc.teamcode;

/**
 * Holds PID values.
 */
public class PID {
    double Propo;
    double Integ;
    double Deriv;

    public PID (double p, double i, double d) {
        Propo = p;
        Integ = i;
        Deriv = d;
    }

    public double getPropo() {
        return Propo;
    }

    public double getInteg() {
        return Integ;
    }

    public double getDeriv() {
        return Deriv;
    }

    @Override
    public String toString() {
        return "Proportional: " + Propo + "\nIntegral: " + Integ + "\nDerivative: " + Deriv;
    }
}
