package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * A utility class that controls all the gunner functions (opening and closing the glyphter, rotating the glypter, etc).
 */
public class GunnerFunction {
    private static final double JEWELPUSHER_SERVO_RETRACTED_POS = 0;
    private static final double JEWELPUSHER_SERVO_EXTENDED_POS = 1;

    private final DcMotor motorRelicSlide;
    private final DcMotor motorWinch;
    private final Servo servoJewelPusher;
    private final TwoStateServo servoGlyphterLeft;
    private final TwoStateServo servoGlyphterRight;
    private final TwoStateServo relicGrabber;
    private final TwoStateServo relicLifter;
    private final TwoStateServo autonGlyphter;
    private final Telemetry telemetry;

    GunnerFunction(HardwareMap hardwareMap, Telemetry telemetry) {
        // Load the needed devices from the hardware map
        this.motorWinch = hardwareMap.dcMotor.get("winchMotor");
        this.motorRelicSlide = hardwareMap.dcMotor.get("relicSlideMotor");
        this.servoGlyphterLeft = new TwoStateServo(hardwareMap.servo.get("glyphterServoLeft"), 0.5, 0, 1, true);
        this.servoGlyphterRight = new TwoStateServo(hardwareMap.servo.get("glyphterServoRight"), 0, 0.5, 1, true);
        this.servoJewelPusher = hardwareMap.servo.get("jewelPusher");
        this.relicGrabber = new TwoStateServo(hardwareMap.servo.get("relicGrabber"), 0, 0.5, 0.01, false);
        this.relicLifter = new TwoStateServo(hardwareMap.servo.get("relicLifter"), 0, 0.8);
        this.autonGlyphter = new TwoStateServo(hardwareMap.servo.get("autonGlyphter"), 0, 0.85);

        this.telemetry = telemetry;

        servoJewelPusher.setPosition(JEWELPUSHER_SERVO_RETRACTED_POS);
    }

    public void upWinch() {
        motorWinch.setPower(0.75);
        telemetry.log().add("Raise Glyphter Winch");
    }

    public void downWinch() {
        motorWinch.setPower(-0.75);
        telemetry.log().add("Lower Glyphter Winch");
    }

    public void stopWinch() {
        motorWinch.setPower(0);
        telemetry.log().add("Stop Glyphter Winch");
    }

    public void openGlyphter() {
        servoGlyphterLeft.passive();
        servoGlyphterRight.passive();
        telemetry.log().add("Open Glyphter");
    }

    public void closeGlyphter() {
        servoGlyphterLeft.active();
        servoGlyphterRight.active();
        telemetry.log().add("Close Glyphter");
    }

    public void openGlyphterIncremental() {
        servoGlyphterLeft.incrementTowardsPassive();
        servoGlyphterRight.incrementTowardsPassive();
        telemetry.log().add("Open Glyphter Incremental");
    }

    public void closeGlyphterIncremental() {
        servoGlyphterLeft.incrementTowardsActive();
        servoGlyphterRight.incrementTowardsActive();
        telemetry.log().add("Close Glyphter Incremental");
    }

    public void extendJewelPusher() { servoJewelPusher.setPosition(JEWELPUSHER_SERVO_EXTENDED_POS);telemetry.log().add("Extend Jewel Pusher"); }

    public void retractJewelPusher() { servoJewelPusher.setPosition(JEWELPUSHER_SERVO_RETRACTED_POS);telemetry.log().add("Retract Jewel Pusher"); }

    public void retractRelicSlide() {
        // TODO: maybe make bigger
        motorRelicSlide.setPower(0.5);
        telemetry.log().add("Expand Relic Slide");
    }

    public void extendRelicSlide() {
        motorRelicSlide.setPower(-0.5);
        telemetry.log().add("Retract Relic Slide");
    }

    public void stopRelicSlide() {
        motorRelicSlide.setPower(0);
        telemetry.log().add("Stop Relic Slide");
    }

    public void toggleRelicGrabber() {
        relicGrabber.toggle();
        telemetry.log().add("Toggle Relic Grabber");
    }

    public void openRelicGrabberIncremental() {
        relicGrabber.incrementTowardsActive();
        telemetry.log().add("Open Relic Grabber Incremental");
    }

    public void closeRelicGrabberIncremental() {
        relicGrabber.incrementTowardsPassive();
        telemetry.log().add("Close Relic Grabber Incremental");
    }

    public void toggleRelicLifter() {
        relicLifter.toggle();
        telemetry.log().add("Toggle Relic Lifter");
    }

    public void raiseRelicLifter() {
        relicLifter.passive();
    }

    public void lowerRelicLifter() {
        relicLifter.active();
    }

    //TODO: Add Auton Glyphter

    public void reset() {
        //closeGlyphter();
        relicGrabber.passive();
        relicLifter.passive();
        servoJewelPusher.setPosition(JEWELPUSHER_SERVO_RETRACTED_POS);
        telemetry.log().add("Reset");
    }
}