package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Servo;

public class TwoStateServo {
    private Servo servo;
    private double passivePosition;
    private double activePosition;

    private boolean isActive;
    private double incrementalSpeed;

    public TwoStateServo(Servo servo, double passivePosition, double activePosition) {
        this(servo, passivePosition, activePosition, 999, false);
    }

    public TwoStateServo(Servo servo, double passivePosition, double activePosition, double incrementalSpeed) {
        this(servo, passivePosition, activePosition, incrementalSpeed, false);
    }

    public TwoStateServo(Servo servo, double passivePosition, double activePosition, double incrementalSpeed, boolean startAsActive) {
        this.servo = servo;
        this.passivePosition = passivePosition;
        this.activePosition = activePosition;
        this.incrementalSpeed = incrementalSpeed;
        isActive = startAsActive;
        updatePosition();
    }

    public void passive() {
        updatePosition(false);
    }

    public void active() {
        updatePosition(true);
    }

    public void toggle() {
        isActive = !isActive;
        updatePosition();
    }

    private void updatePosition() {
        servo.setPosition(isActive ? activePosition : passivePosition);
    }

    private void updatePosition(boolean isActive) {
        this.isActive = isActive;
        updatePosition();
    }

    public void incrementTowardsPassive() {
        incrementTowards(passivePosition);
    }

    public void incrementTowardsActive() {
        incrementTowards(activePosition);
    }

    private void incrementTowards(double desiredValue) {
        double servoPos = servo.getPosition();
        if (servoPos > desiredValue) {
            servo.setPosition(RobotUtil.clipRange(passivePosition, activePosition, servoPos + incrementalSpeed));
        } else {
            servo.setPosition(RobotUtil.clipRange(passivePosition, activePosition, servoPos - incrementalSpeed));
        }
    }

    public Servo getServo() {
        return servo;
    }
}
