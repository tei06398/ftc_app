package org.firstinspires.ftc.teamcode.util.drive;

/**
 * Builds DiagonalVectorSteering objects, implementing movement logic such as strafe, pivot, and rotate.
 */
public class DiagonalVectorSteeringBuilder implements SteeringBuilder<DiagonalVectorSteering> {
    private double lf = 0;
    private double lb = 0;
    private double rf = 0;
    private double rb = 0;

    public DiagonalVectorSteeringBuilder() {
    }

    /* UTILITIES */

    public DiagonalVectorSteeringBuilder setAllPowers(double power) {
        lf = power;
        lb = power;
        rf = power;
        rb = power;

        return this;
    }

    /**
     * Add a certain power to each motor.
     * @param power The power to add.
     */
    public DiagonalVectorSteeringBuilder addToAllPowers(double power) {
        lf += power;
        lb += power;
        rf += power;
        rb += power;

        return this;
    }

    /* LINEAR MOVEMENT */

    public DiagonalVectorSteeringBuilder strafeRadians(double angle) {
        return strafeRadians(angle, 1);
    }

    /**
     * Strafe the robot in any direction, at a certain power.
     * @param angle Angle, specified in radians where 0 is right.
     * @param power The power of the strafe.
     */
    public DiagonalVectorSteeringBuilder strafeRadians(double angle, double power) {
        // This "fixes" a really annoying bug where the left and right controls are inverted. We don't know where it
        // is, so we just inverted the angle by reflecting it over the y-axis. Maybe we will fix this bug next year.
        if (angle >= 0) {
            angle = Math.PI - angle;
        } else {
            angle = -Math.PI - angle;
        }

        double speedX = Math.cos(angle - Math.toRadians(45));
        double speedY = Math.sin(angle - Math.toRadians(45));

        // so there's always going to be a speed that's plus or minus 1
        double divider = Math.max(Math.abs(speedX), Math.abs(speedY));

        lf += speedX / divider * power;
        rb -= speedX / divider * power;
        lb += speedY / divider * power;
        rf -= speedY / divider * power;

        return this;
    }

    /**
     * Strafe in any direction.
     * @param angle The angle of the direction, in degrees.
     */
    public DiagonalVectorSteeringBuilder strafe(double angle) {
        return strafeRadians(Math.toRadians(angle));
    }

    public DiagonalVectorSteeringBuilder strafe(double angle, double power) {
        return strafeRadians(Math.toRadians(angle), power);
    }

    public DiagonalVectorSteeringBuilder strafeDegrees(double angle) {
        return strafe(angle);
    }

    public DiagonalVectorSteeringBuilder strafeDegrees(double angle, double power) {
        return strafe(angle, power);
    }

    /* ROTATION */

    public DiagonalVectorSteeringBuilder turn(boolean isClockwise, double power) {
        return addToAllPowers(isClockwise ? power : -power);
    }

    public DiagonalVectorSteeringBuilder turn(double power) {
        return addToAllPowers(power);
    }

    public DiagonalVectorSteeringBuilder turnClockwise(double power) {
        return turn(true, power);
    }

    public DiagonalVectorSteeringBuilder turnCounterclockwise(double power) {
        return turn(false, power);
    }

    public DiagonalVectorSteeringBuilder turn(boolean isClockwise) {
        return turn(isClockwise, 1);
    }

    public DiagonalVectorSteeringBuilder turnClockwise() {
        return turnClockwise(1);
    }

    public DiagonalVectorSteeringBuilder turnCounterclockwise() {
        return turnCounterclockwise(1);
    }

    /* MISC */

    /**
     * Pivot around the point.
     * @param isClockwise Whether to pivot clockwise.
     * @param rotationWeight The weight given to rotation, where one unit of weight is already given to strafing.
     */
    public DiagonalVectorSteeringBuilder pivot(boolean isClockwise, double rotationWeight) {
        strafeDegrees(isClockwise ? 180 : 0);
        turn(isClockwise, rotationWeight);

        return this;
    }

    @Override
    public DiagonalVectorSteering build() {
        DiagonalVectorSteering result = new DiagonalVectorSteering(lf, lb, rf, rb);
        noMotion();
        return result;
    }

    @Override
    public DiagonalVectorSteeringBuilder noMotion() {
        return setAllPowers(0);
    }
}
