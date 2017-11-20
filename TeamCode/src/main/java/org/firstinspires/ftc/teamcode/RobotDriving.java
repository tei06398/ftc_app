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
    private DcMotor motorLF;
    private DcMotor motorLB;
    private DcMotor motorRF;
    private DcMotor motorRB;
    private Telemetry telemetry;
    public static final double MAX_SPEED_RATIO = 1;
    public static final double MIN_SPEED_RATIO = 0.35;

    // The big problem with this is that someone might scramble up the motor order (ie. put the LB motor in the LF argument slot)
    // and it would be a pain to debug.
    public RobotDriving(DcMotor LF, DcMotor LB, DcMotor RF, DcMotor RB, Telemetry telemetry) {
        motorLF = LF;
        motorLB = LB;
        motorRF = RF;
        motorRB = RB;
        this.telemetry = telemetry;
    }

    public Steering getSteering() {
        return new Steering();
    }
    
    public TimedSteering getTimedSteering() {
        return new TimedSteering();
    }
    
    public void stopAllMotors() {
        motorLF.setPower(0);
        motorLB.setPower(0);
        motorRF.setPower(0);
        motorRF.setPower(0);
        motorRB.setPower(0);
    }
    
    // Easy test for the motors. The test should make the robot spin clockwise for 5 seconds, then counterclockwise for 5 seconds.
    public void testMotors() {
        motorLF.setPower(1);
        motorLB.setPower(1);
        motorRF.setPower(1);
        motorRB.setPower(1);

        try {
            wait(5);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        motorLF.setPower(-1);
        motorLB.setPower(-1);
        motorRF.setPower(-1);
        motorRB.setPower(-1);

        try {
            wait(5);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }
    
    /*public void wait(double seconds) {
        try {
            Thread.sleep((long) (seconds * 1000));
        } catch (InterruptedException e) {
            // maybe have a better exception handling system?
            e.printStackTrace();
        }
    }*/
    
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
                motorLF.setPower(powerLF / maxRawPower * speedRatio);
                motorLB.setPower(powerLB / maxRawPower * speedRatio);
                motorRF.setPower(powerRF / maxRawPower * speedRatio);
                motorRB.setPower(powerRB / maxRawPower * speedRatio);
            } else {
                stopAllMotors();
            }
            
            setAllPowers(0);
        }
    }
    
    // An inner class that handles _timed_ steering: ie. "move forward for 3 seconds, then stop" vs. "move forward until you get the next command"
    // Every movement is finished when it is created. There's no need to call "finishSteering()." However, this means that this class can't do two motions
    // at once (ie. move AND rotate).
    public class TimedSteering extends Steering {
        private double time = 0;
        
        public TimedSteering() {
            super();
        }
        
        /* LINEAR MOVEMENT */
        
        public void moveRadians(double angle, double time) {
            super.moveRadians(angle);
            this.time = time;
            finishSteering();
        }
        
        public void move(double angle, double time) {
            super.move(angle);
            this.time = time;
            finishSteering();
        }
        
        public void moveDegrees(double angle, double time) {
            move(angle, time);
        }
        
        public void right(double time) { move(0, time); finishSteering(); }
        public void forward(double time) { move(90, time); finishSteering(); }
        public void left(double time) { move(180, time); finishSteering(); }
        public void backward(double time) { move(270, time); finishSteering(); }
        
        /* ROTATION */
        
        public void turn(boolean isClockwise, double time) {
            super.turn(isClockwise);
            this.time = time;
            finishSteering();
        }
        
        public void turnClockwise(double time) {
            super.turnClockwise();
            this.time = time;
            finishSteering();
        }
        
        public void turnCounterclockwise(double time) {
            super.turnCounterclockwise();
            this.time = time;
            finishSteering();
        }
        
        /* MISC */
        
        // NO NEED TO CALL THIS DIRECTLY. It is automatically done after every steering operation with a time. 
        public void finishSteering() {
            super.finishSteering();
            
            if (time != 0) {
                // You don't need any exception handling here because wait(time) "converts"
                // an InterruptedException to a RuntimeException. :)
                /*try {
                    wait((long) time);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }*/

                // Stop all the motors at the end of the motion.
                stopAllMotors();
                
                // Reset time.
                time = 0;
            }
        }
    }
}
