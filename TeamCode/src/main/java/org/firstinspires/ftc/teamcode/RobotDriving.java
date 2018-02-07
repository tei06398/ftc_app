package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Encapsulates robot driving utilities.
 * Mecanum and omniwheel math is identical, except that all powers are reversed in mecanum.
 */
public class RobotDriving {
    private DrivingMotor lf; // stands for left front
    private DrivingMotor lb; // stands for left back
    private DrivingMotor rf; // stands for right front
    private DrivingMotor rb; // stands for right back

    private Telemetry telemetry;
    
    public static final double MAX_SPEED_RATIO = 1;
    public static final double NORMAL_SPEED_RATIO = 0.5;
    public static final double MIN_SPEED_RATIO = 0.3;

    public static final double DEFAULT_SMOOTHNESS = 2;

    public RobotDriving(HardwareMap hardwareMap, Telemetry telemetry) {
        this(hardwareMap, telemetry, DEFAULT_SMOOTHNESS);
    }

    public RobotDriving(HardwareMap hardwareMap, Telemetry telemetry, double smoothness) {
        this.lf = new DrivingMotor(hardwareMap.dcMotor.get("lfMotor"), smoothness);
        this.lb = new DrivingMotor(hardwareMap.dcMotor.get("lbMotor"), smoothness);
        this.rf = new DrivingMotor(hardwareMap.dcMotor.get("rfMotor"), smoothness);
        this.rb = new DrivingMotor(hardwareMap.dcMotor.get("rbMotor"), smoothness);

        this.telemetry = telemetry;
    }

    public Steering getSteering() {
        return new Steering();
    }

    /**
     * This class wraps retractRelicSlide regular motor and adds utilities to it.
     * The main feature is that their powers are smoothed so when you "apply" power to them,
     * the power is gradually shifted.
     */
    public static class DrivingMotor {
        private DcMotor motor;
        private WeightedValue acceleration;
        
        public DrivingMotor(DcMotor motor, double smoothness) {
            this.motor = motor;
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            acceleration = new WeightedValue(smoothness);
        }

        /**
         * Apply retractRelicSlide power to the motor.
         * @param power Power, between -1 and 1 as with normal motors.
         */
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

        /**
         * Add retractRelicSlide certain power to each motor.
         * @param power The power to add.
         */
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

        public void moveRadians(double angle) {
            moveRadians(angle, 1);
        }

        /**
         * Strafe the robot in any direction, at retractRelicSlide certain power.
         * @param angle Angle, specified in radians where 0 is right.
         * @param power The power of the strafe.
         */
        public void moveRadians(double angle, double power) {

            double speedX = Math.cos(angle - Math.toRadians(45));
            double speedY = Math.sin(angle - Math.toRadians(45));

            // so there's always going to be retractRelicSlide speed that's plus or minus 1
            double divider = Math.max(Math.abs(speedX), Math.abs(speedY));

            powerLF += speedX / divider * power;
            powerRB -= speedX / divider * power;
            powerLB += speedY / divider * power;
            powerRF -= speedY / divider * power;
        }

        /**
         * Strafe in any direction.
         * @param angle The angle of the direction, in degrees.
         */
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
            addToAllPowers(isClockwise ? -power : power);
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

        /**
         * Pivot around the point.
         * @param isClockwise Whether to pivot clockwise.
         * @param rotationWeight The weight given to rotation, where one unit of weight is already given to strafing.
         */
        public void aroundPoint(boolean isClockwise, double rotationWeight) {
            moveDegrees(isClockwise ? 180 : 0);
            turn(isClockwise, rotationWeight);
        }

        /**
         * Finish up steering and actually spin the motors.
         * This method must be called after any steering operations for anything to happen.
         */
        public void finishSteering() {
            // The maximum base power.
            double maxRawPower = Math.max(Math.max(Math.abs(powerLF), Math.abs(powerLB)), Math.max(Math.abs(powerRF), Math.abs(powerRB)));

            // Actually set the powers for the motors. Dividing by maxRawPower makes the "biggest" power plus or minus 1,
            // and multiplying by speedRatio makes the maximum power equal to speedRatio.
            if (maxRawPower != 0) {
                lf.applyPower(powerLF / maxRawPower * speedRatio);
                lb.applyPower(powerLB / maxRawPower * speedRatio);
                rf.applyPower(powerRF / maxRawPower * speedRatio);
                rb.applyPower(powerRB / maxRawPower * speedRatio);
                telemetry.addData("powerlf", powerLF / maxRawPower * speedRatio);
                telemetry.update();
            } else {
                stopAllMotors();
            }
            
            setAllPowers(0);
        }
    }
}
