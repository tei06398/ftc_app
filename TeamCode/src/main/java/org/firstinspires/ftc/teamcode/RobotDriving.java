package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

public class RobotDriving {
    private DcMotor motorLF = null;
    private DcMotor motorLB = null;
    private DcMotor motorRF = null;
    private DcMotor motorRB = null;
    final double MAX_SPEED_RATIO = 1;
    final double MIN_SPEED_RATIO = 0.35;

    public RobotDriving() { //In case you still want the default constructor, for some reason, Jeffrey

    }

    public RobotDriving(DcMotor LF, DcMotor LB, DcMotor RF, DcMotor RB) {
        motorLF = LF;
        motorLB = LB;
        motorRF = RF;
        motorRB = RB;
    }

    public Steering getSteering() {
        if (motorLF == null || motorLB == null || motorRF == null || motorRB == null)
            throw new IllegalStateException("All motors must be set before creating a Steering object");
        return new Steering();
    }
    
    public void stopAllMotors() {
        motorLF.setPower(0);
        motorLB.setPower(0);
        motorRF.setPower(0);
        motorRB.setPower(0);
    }
    
    public void wait(double seconds) {
        try {
            Thread.sleep((long)seconds * 1000);
        } catch (InterruptedException e) {
            // maybe have a better exception handling system?
            throw new RuntimeException("Thread interrupted at RobotDriving.Steering.finishSteering()");
        }
    }
    
    // An inner class that manages the repeated recalculation of motor powers.
    public class Steering {
        private double powerLF = 0;
        private double powerLB = 0;
        private double powerRF = 0;
        private double powerRB = 0;
        private double speedRatio = MAX_SPEED_RATIO;
        private double time;

        public Steering() {
        }

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

        public void turn(boolean isClockwise) {
            addToAllPowers(isClockwise ? 1 : -1);
        }

        public void turnClockwise() {
            turn(true);
        }

        public void turnCounterclockwise() {
            turn(false);
        }

        // **Angle is in radians, not degrees.**
        public void moveRadians(double angle) {
            double speedX = Math.cos(angle - Math.PI / 4);
            double speedY = Math.sin(angle - Math.PI / 4);

            // so there's always going to be a speed that's +-1
            double divider = Math.max(Math.abs(speedX), Math.abs(speedY));

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
        
        /* Utilities for those who don't want to calculate angles */

        public void right() {
            move(0);
        }

        public void up() {
            move(90);
        }

        public void left() {
            move(180);
        }

        public void down() {
            move(270);
        }
        
        public void setTime(double time) {
            this.time = time;
        }
        
        // Actually makes the motors spin. You must call this once all the movements are set for anything to happen.
        public void finishSteering() {
            // The maximum base power.
            double maxRawPower = Math.max(Math.max(Math.abs(powerLF), Math.abs(powerLB)), Math.max(Math.abs(powerRF), Math.abs(powerRB)));

            // Now, actually set the powers for the motors. Dividing by maxRawPower makes the "biggest" power +-1, and multiplying by speedRatio
            // makes the maximum power speedRatio.
            if (maxRawPower != 0) {
                motorLF.setPower(powerLF / maxRawPower * speedRatio);
                motorLB.setPower(powerLB / maxRawPower * speedRatio);
                motorRF.setPower(powerRF / maxRawPower * speedRatio);
                motorRB.setPower(powerRB / maxRawPower * speedRatio);
            } else {
                stopAllMotors();
            }
            
            if (time != 0) {
                try {
                    wait((long)time);
                } catch (InterruptedException e) {
                    throw new RuntimeException("Thread interrupted at RobotDriving.Steering.finishSteering()");
                }

                // stop all the motors at the end of the motion.
                stopAllMotors();
            }
        }
    }
}
