package org.firstinspires.ftc.teamcode.util.drive;

/**
 * Implements steering that uses motors placed in a diagonal (45 degree) vector layout.
 */
public class DiagonalVectorSteering implements Steering<DiagonalVectorSteering> {
    private final double lf;

    private final double lb;
    private final double rf;
    private final double rb;
    public DiagonalVectorSteering(double lf, double lb, double rf, double rb) {
        this.lf = lf;
        this.lb = lb;
        this.rf = rf;
        this.rb = rb;
    }

    @Override
    public DiagonalVectorSteering scale(double maxPower) {
        double maxRawPower = Math.max(Math.max(Math.abs(lf), Math.abs(lb)), Math.max(Math.abs(rf), Math.abs(rb)));

        // Dividing by maxRawPower makes the "biggest" power plus or minus 1,
        // and multiplying by ceiling makes the maximum power equal to ceiling.
        if (maxRawPower == 0) {
            return new DiagonalVectorSteering(0, 0, 0, 0);
        } else {
            return new DiagonalVectorSteering(lf / maxRawPower * maxPower,
                    rf / maxRawPower * maxPower,
                    rf / maxRawPower * maxPower,
                    rb / maxRawPower * maxPower);
        }
    }

    public double getLf() {
        return lf;
    }

    public double getLb() {
        return lb;
    }

    public double getRf() {
        return rf;
    }

    public double getRb() {
        return rb;
    }
}
