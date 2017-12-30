package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
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
    
    public static final double MAX_SPEED_RATIO = 1;
    public static final double NORMAL_SPEED_RATIO = 0.5;
    public static final double MIN_SPEED_RATIO = 0.3;
    
    public static final double DEFAULT_SMOOTHNESS = 0.1;
    
    public RobotDriving(HardwareMap hardwareMap, Telemetry telemetry, double smoothness) {
        this.lf = new DrivingMotor(hardwareMap.dcMotor.get("lfMotor"), smoothness);
        this.lb = new DrivingMotor(hardwareMap.dcMotor.get("lbMotor"), smoothness);
        this.rf = new DrivingMotor(hardwareMap.dcMotor.get("rfMotor"), smoothness);
        this.rb = new DrivingMotor(hardwareMap.dcMotor.get("rbMotor"), smoothness);
        
        this.telemetry = telemetry;
    }

    public RobotDriving(HardwareMap hardwareMap, Telemetry telemetry) {
        this(hardwareMap, telemetry, DEFAULT_SMOOTHNESS);
    }
    
    public RobotDriving(DcMotor LF, DcMotor LB, DcMotor RF, DcMotor RB, Telemetry telemetry) {
        this(LF, LB, RF, RB, telemetry, DEFAULT_SMOOTHNESS);
    }

    public RobotDriving(DcMotor LF, DcMotor LB, DcMotor RF, DcMotor RB, Telemetry telemetry, double smoothness) {
        this.lf = new DrivingMotor(LF, smoothness);
        this.lb = new DrivingMotor(LB, smoothness);
        this.rf = new DrivingMotor(RF, smoothness);
        this.rb = new DrivingMotor(RB, smoothness);
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
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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

        private double speedRatio = NORMAL_SPEED_RATIO;

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
            moveRadians(angle, 1);
        }

        public void moveRadians(double angle, double power) {
            // This "fixes" a really annoying bug where the left and right controls are inverted. We don't know where it
            // is, so we just inverted the angle by reflecting it over the y-axis.
            if (angle >= 0) {
                angle = Math.PI - angle;
            } else {
                angle = -Math.PI - angle;
            }

            double speedX = Math.cos(angle - Math.toRadians(45));
            double speedY = Math.sin(angle - Math.toRadians(45));

            // so there's always going to be a speed that's +-1
            double divider = Math.max(Math.abs(speedX), Math.abs(speedY));

            powerLF += speedX / divider * power;
            powerRB -= speedX / divider * power;
            powerLB += speedY / divider * power;
            powerRF -= speedY / divider * power;
        }

        // **Angle is in degrees, not radians.**
        public void move(double angle) {
            moveRadians(Math.toRadians(angle));
        }

        public void move(double angle, double power) {
            moveRadians(Math.toRadians(angle), power);
        }

        public void moveDegrees(double angle) {
            move(angle);
        }

        public void moveDegrees(double angle, double power) {
            move(angle, power);
        }

        /* ROTATION */

        public void turn(boolean isClockwise, double power) {
            addToAllPowers(isClockwise ? power : -power);
        }

        public void turn(double power) { addToAllPowers(power);}

        public void turnClockwise(double power) {
            turn(true, power);
        }

        public void turnCounterclockwise(double power) {
            turn(false, power);
        }

        public void turn(boolean isClockwise) {
            turn(isClockwise, 1);
        }

        public void turnClockwise() {
            turnClockwise(1);
        }

        public void turnCounterclockwise() {
            turnCounterclockwise(1);
        }

        /* MISC */

        public void aroundPoint(boolean isClockwise, double rotationWeight) {
            moveDegrees(isClockwise ? 180 : 0);
            turn(isClockwise, rotationWeight);
        }

        // Actually makes the motors spin. You must call this once all the movements are set for anything to happen.
        public void finishSteering() {
            // The maximum base power.
            double maxRawPower = Math.max(Math.max(Math.abs(powerLF), Math.abs(powerLB)), Math.max(Math.abs(powerRF), Math.abs(powerRB)));

            // Now, actually set the powers for the motors. Dividing by maxRawPower makes the "biggest" power +-1, and multiplying by speedRatio
            // makes the maximum power speedRatio.
            if (maxRawPower != 0) {
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
