public class RobotDriving {
    public DcMotor motorLF = null;
    public DcMotor motorLB = null;
    public DcMotor motorRF = null;
    public DcMotor motorRB = null;
    
    public RobotDriving() {
        final double MAX_SPEED_RATIO = 1;
        final double MIN_SPEED_RATIO = 0.35;
    }
    
    public Steering getSteering() {
        if (motorLF == null || motorLB == null || motorRF == null || motorRB == null) throw new IllegalStateException("All motors must be set before creating a Steering object");
        
        return new Steering();
    }
    
    // An inner class that manages the repeated recalculation of motor powers.
    public class Steering {
        public double powerLF = 0;
        public double powerLB = 0;
        public double powerRF = 0;
        public double powerRB = 0;
        public double speedRatio = MAX_SPEED_RATIO;
        
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
        
        // Actually makes the motors spin. You must call this once all the movements are set for anything to happen.
        public void finishSteering() {
            // The maximum base power.
            double maxRawPower = Math.max(Math.max(Math.abs(powerLF), Math.abs(powerLB)), Math.max(Math.abs(powerRF), Math.abs(powerRB)));

            // Now, actually set the powers for the motors. Dividing by maxRawPower makes the "biggest" power +-1, and multiplying by SPEED_RATIO
            // makes the maximum power SPEED_RATIO.
            if (maxRawPower != 0) {
                motorLF.setPower(powerLF / maxRawPower * SPEED_RATIO);
                motorLB.setPower(powerLB / maxRawPower * SPEED_RATIO);
                motorRF.setPower(powerRF / maxRawPower * SPEED_RATIO);
                motorRB.setPower(powerRB / maxRawPower * SPEED_RATIO);
            } else {
                motorLF.setPower(0);
                motorLB.setPower(0);
                motorRF.setPower(0);
                motorRB.setPower(0);
            }
        }
