package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * A utility class that controls all the gunner functions (opening and closing the glyphter, rotating the glypter, etc).
 */
public class GunnerFunction {
    // TODO: These servos are probably NOT continuous rotation! Change numbers.
    private static final double JEWELPUSHER_SERVO_UP_POS = 1;
    private static final double JEWELPUSHER_SERVO_DOWN_POS = 0;

    private final DcMotor motorRelicSlide;
    private final DcMotor motorWinch;
    private final Servo servoJewelPusher;
    private final TwoStateServo servoGlyphterLeft;
    private final TwoStateServo servoGlyphterRight;
    private final TwoStateServo relicGrabber;
    private final TwoStateServo relicLifter;
    private final Telemetry telemetry;

    GunnerFunction(HardwareMap hardwareMap, Telemetry telemetry) {
        // Load the needed devices from the hardware map
        this.motorWinch = hardwareMap.dcMotor.get("winchMotor");
        this.motorRelicSlide = hardwareMap.dcMotor.get("relicSlideMotor");
        this.servoGlyphterLeft = new TwoStateServo(hardwareMap.servo.get("glyphterServoLeft"), 0, 0.5, 1, true);
        this.servoGlyphterRight = new TwoStateServo(hardwareMap.servo.get("glyphterServoRight"), 0, 0.5, 1, true);
        this.servoJewelPusher = hardwareMap.servo.get("jewelPusher");
        this.relicGrabber = new TwoStateServo(hardwareMap.servo.get("relicGrabber"), 0, 0.5);
        this.relicLifter = new TwoStateServo(hardwareMap.servo.get("relicLifter"), 0, 0.5);

        this.telemetry = telemetry;

        servoJewelPusher.setPosition(JEWELPUSHER_SERVO_UP_POS);
    }

    public void upWinch() {
        motorWinch.setPower(0.75);
    }

    public void downWinch() {
        motorWinch.setPower(-0.75);
    }

    public void stopWinch() {
        motorWinch.setPower(0);
    }

    public void openGlyphter() {
        servoGlyphterLeft.passive();
        servoGlyphterRight.passive();
    }

    public void closeGlyphter() {
        servoGlyphterLeft.active();
        servoGlyphterRight.active();
    }

    public void openGlyphterIncremental() {
        servoGlyphterLeft.incrementTowardsPassive();
        servoGlyphterRight.incrementTowardsPassive();
    }

    public void closeGlyphterIncremental() {
        servoGlyphterLeft.incrementTowardsActive();
        servoGlyphterRight.incrementTowardsActive();
    }

    public void lowerJewelPusher() { servoJewelPusher.setPosition(JEWELPUSHER_SERVO_DOWN_POS); }

    public void raiseJewelPusher() { servoJewelPusher.setPosition(JEWELPUSHER_SERVO_UP_POS); }

    public void expandRelicSlide() {
        // TODO: maybe make bigger
        motorRelicSlide.setPower(0.1);
    }

    public void retractRelicSlide() {
        motorRelicSlide.setPower(-0.1);
    }

    public void stopRelicSlide() {
        motorRelicSlide.setPower(0);
    }

    public void toggleRelicGrabber() {
        relicGrabber.toggle();
    }

    public void toggleRelicGrabberPivot() {
        relicLifter.toggle();
    }

    public void reset() {
        closeGlyphter();
        relicGrabber.passive();
        relicLifter.passive();
        servoJewelPusher.setPosition(JEWELPUSHER_SERVO_UP_POS);
    }
}