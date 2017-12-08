package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/*
    A utility class for robot driving.
    Use it like this:
    RobotDriving rd = new RobotDriving();
    rd.setMotorLF( [motor object] );
    rd.setMotorLB( [motor object] );
    ...
    
    RobotDriving.TimedSteering ts = rd.getSteering();
    
    ts.forward(1); // forward for 1 sec
    ts.turnClockwise(0.2); // clockwise for 2 sec
    ts.move(30, 2); // move at 30 degree angle for 2 sec
*/
public class RobotDriving {
    // DrivingMotors are not real motors. They are a utility class that wraps over a motor object,
    // and the main feature is that their powers are weighted so when you "apply" power to them,
    // the power is gradually shifted.
    private DrivingMotor lf;
    private DrivingMotor lb;
    private DrivingMotor rf;
    private DrivingMotor rb;

    private Telemetry telemetry;
    public static final double MAX_SPEED_RATIO = 0.5;
    public static final double MIN_SPEED_RATIO = 0.3;
    public static final double SMOOTHNESS = 0.1;
    
    public RobotDriving(DcMotor LF, DcMotor LB, DcMotor RF, DcMotor RB, Telemetry telemetry) {
        this.lf = new DrivingMotor(LF, SMOOTHNESS);
        this.lb = new DrivingMotor(LB, SMOOTHNESS);
        this.rf = new DrivingMotor(RF, SMOOTHNESS);
        this.rb = new DrivingMotor(RB, SMOOTHNESS);
        this.telemetry = telemetry;
    }

    public Steering getSteering() {
        return new Steering();
    }
    
    /*public void wait(double seconds) {
        try {
            Thread.sleep((long) (seconds * 1000));
        } catch (InterruptedException e) {
            // maybe have a better exception handling system?
            e.printStackTrace();
        }
    }*/
    
    // Encapsulates a motor and all its utilities.
    public static class DrivingMotor {
        private DcMotor motor;
        private WeightedValue acceleration;
        
        public DrivingMotor(DcMotor motor, double smoothness) {
            this.motor = motor;
            acceleration = new WeightedValue(smoothness);
        }
        
        public void applyPower(double power) {
            this.motor.setPower(acceleration.applyValue(power));
        }
    }
    
    // An inner class that manages the repeated recalculation of motor powers.
    public class Steering {
        private double powerLF = 0;
        private double powerLB = 0;
        private double powerRF = 0;
        private double powerRB = 0;

        private double speedRatio = MAX_SPEED_RATIO;

        public Steering() {
        }

        /* UTILITIES */

        public void setSpeedRatio(double speedRatio) {
            this.speedRatio = speedRatio;
        }

        public void setAllPowers(double power) {
            powerLF = power;
            powerLB = power;
            powerRF = power;
            powerRB = power;
        }

        public void addToAllPowers(double power) {
            powerLF += power;
            powerLB += power;
            powerRF += power;
            powerRB += power;
        }
        
        public void stopAllMotors() {
            lf.applyPower(0);
            lb.applyPower(0);
            rf.applyPower(0);
            rb.applyPower(0);
        }

        /* LINEAR MOVEMENT */

        // **Angle is in radians, not degrees.**
        public void moveRadians(double angle) {

            // This "fixes" a really annoying bug where the left and right controls are inverted. We don't know where it
            // is, so we just inverted the angle by reflecting it over the y-axis.
            if (angle >= 0) {
                angle = Math.PI - angle;
            } else {
                angle = -Math.PI - angle;
            }

            double speedX = Math.cos(angle - Math.toRadians(45));
            double speedY = Math.sin(angle - Math.toRadians(45));

            telemetry.addData("speed x: ", speedX);
            telemetry.addData("speed y: ", speedY);

            // so there's always going to be a speed that's +-1
            double divider = Math.max(Math.abs(speedX), Math.abs(speedY));
            telemetry.addData("divider: ", divider);
            powerLF += speedX / divider;
            powerRB -= speedX / divider;
            powerLB += speedY / divider;
            powerRF -= speedY / divider;
        }

        // **Angle is in degrees, not radians.**
        public void move(double angle) {
            moveRadians(Math.toRadians(angle));
        }

        public void moveDegrees(double angle) {
            move(angle);
        }

        /* ROTATION */

        public void turn(boolean isClockwise) {
            addToAllPowers(isClockwise ? 1 : -1);
        }

        public void turnClockwise() {
            turn(true);
        }

        public void turnCounterclockwise() {
            turn(false);
        }

        /* MISC */

        // Actually makes the motors spin. You must call this once all the movements are set for anything to happen.
        public void finishSteering() {
            // The maximum base power.
            double maxRawPower = Math.max(Math.max(Math.abs(powerLF), Math.abs(powerLB)), Math.max(Math.abs(powerRF), Math.abs(powerRB)));

            // Now, actually set the powers for the motors. Dividing by maxRawPower makes the "biggest" power +-1, and multiplying by speedRatio
            // makes the maximum power speedRatio.
            if (maxRawPower != 0) {
                telemetry.addData("power lf: ", powerLF);
                telemetry.addData("power lb: ", powerLB);
                telemetry.addData("power rf: ", powerRF);
                telemetry.addData("power rb: ", powerRB);

                telemetry.addData("max raw power: ", maxRawPower);
                
                lf.applyPower(powerLF / maxRawPower * speedRatio);
                lb.applyPower(powerLB / maxRawPower * speedRatio);
                rf.applyPower(powerRF / maxRawPower * speedRatio);
                rb.applyPower(powerRB / maxRawPower * speedRatio);
            } else {
                stopAllMotors();
            }
            
            setAllPowers(0);
        }
    }
}
