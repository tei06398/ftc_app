package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

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
    private DcMotor motorLF = null;
    private DcMotor motorLB = null;
    private DcMotor motorRF = null;
    private DcMotor motorRB = null;
    final double MAX_SPEED_RATIO = 1;
    final double MIN_SPEED_RATIO = 0.35;
    
    public void setMotorLF(DcMotor motorLF) { this.motorLF = motorLF; }
    public void setMotorLB(DcMotor motorLB) { this.motorLB = motorLB; }
    public void setMotorRF(DcMotor motorRF) { this.motorRF = motorLB; }
    public void setMotorRB(DcMotor motorRB) { this.motorRB = motorLB; }
    
    public RobotDriving() { //In case you still want the default constructor, for some reason, Jeffrey

    }
    
    // The big problem with this is that someone might scramble up the motor order (ie. put the LB motor in the LF argument slot)
    // and it would be a pain to debug.
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
    
    public TimedSteering getTimedSteering() {
        if (motorLF == null || motorLB == null || motorRF == null || motorRB == null)
            throw new IllegalStateException("All motors must be set before creating a Steering object");
        return new TimedSteering();
    }
    
    public void stopAllMotors() {
        motorLF.setPower(0);
        motorLB.setPower(0);
        motorRF.setPower(0);
        motorRB.setPower(0);
    }
    
    // Easy test for the motors. The test should make the robot spin clockwise for 5 seconds, then counterclockwise for 5 seconds.
    public void testMotors() {
        motorLF.setPower(1);
        motorLB.setPower(1);
        motorRF.setPower(1);
        motorRB.setPower(1);
        
        wait(5);
        
        motorLF.setPower(-1);
        motorLB.setPower(-1);
        motorRF.setPower(-1);
        motorRB.setPower(-1);
        
        wait(5);
    }
    
    public void wait(double seconds) {
        try {
            Thread.sleep((long) (seconds * 1000));
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
                wait(time);

                // Stop all the motors at the end of the motion.
                stopAllMotors();
                
                // Reset time.
                time = 0;
            }
        }
    }
}
