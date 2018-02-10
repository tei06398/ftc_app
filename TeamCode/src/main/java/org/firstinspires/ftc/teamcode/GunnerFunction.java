package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * A utility class that controls all the gunner functions (opening and closing the glyphter, rotating the glypter, etc).
 */
public class GunnerFunction {
    private static final double JEWELPUSHER_SERVO_RETRACT_POS = 0.2;
    private static final double JEWELPUSHER_SERVO_EXTEND_POS = 0.8;
    private static final double JEWELPUSHER_SERVO_STOPPED_POS = 0.5;

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
        ServoController servoController = hardwareMap.servoController.get("Servo Controller 1");
        servoController.pwmDisable();
        this.motorWinch = hardwareMap.dcMotor.get("winchMotor");
        this.motorRelicSlide = hardwareMap.dcMotor.get("relicSlideMotor");
        this.servoGlyphterLeft = new TwoStateServo(hardwareMap.servo.get("glyphterServoLeft"), 0, 0.5, 1, true);
        this.servoGlyphterRight = new TwoStateServo(hardwareMap.servo.get("glyphterServoRight"), 1, 0.5, 1, true);
        this.servoJewelPusher = hardwareMap.servo.get("jewelPusher");
        servoJewelPusher.setPosition(JEWELPUSHER_SERVO_STOPPED_POS);
        this.relicGrabber = new TwoStateServo(hardwareMap.servo.get("relicGrabber"), 0, 0.6, 0.02, false);
        this.relicLifter = new TwoStateServo(hardwareMap.servo.get("relicLifter"), 0, 1, 0.01);
        this.autonGlyphter = new TwoStateServo(hardwareMap.servo.get("autonGlyphter"), 0, 1);

        this.telemetry = telemetry;

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

    public void openGlyphterFully() {
        servoGlyphterLeft.getServo().setPosition(0);
        servoGlyphterRight.getServo().setPosition(1);
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

    public void extendJewelPusher() { servoJewelPusher.setPosition(JEWELPUSHER_SERVO_EXTEND_POS);telemetry.log().add("Extend Jewel Pusher"); }

    public void retractJewelPusher() { servoJewelPusher.setPosition(JEWELPUSHER_SERVO_RETRACT_POS);telemetry.log().add("Retract Jewel Pusher"); }

    public void stopJewelPusher() { servoJewelPusher.setPosition(JEWELPUSHER_SERVO_STOPPED_POS);telemetry.log().add("Stop Jewel Pusher");}

    public void retractRelicSlide() {
        // TODO: maybe make bigger
        motorRelicSlide.setPower(0.75);
        telemetry.log().add("Expand Relic Slide");
    }

    public void extendRelicSlide() {
        motorRelicSlide.setPower(-0.75);
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

    public void releaseRelic() {
        relicGrabber.incrementTowardsActive();
        if (relicLifter.getServo().getPosition() > 0.7) {
            relicLifter.incrementTowardsPassive();
        }
        telemetry.log().add("Release Relic");
    }

    public void grabRelic() {
        relicGrabber.incrementTowardsPassive();
        motorRelicSlide.setPower(-0.3);
        telemetry.log().add("Grab Relic");
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

    public void extendAutonGlyphter() {
        autonGlyphter.active();
    }

    public void retractAutonGlyphter() {
        autonGlyphter.passive();
    }

    public void stopAutonGlyphter() {
        autonGlyphter.getServo().setPosition(0.5);
    }

    public void reset() {
        //closeGlyphter();
        relicGrabber.passive();
        relicLifter.passive();
        servoJewelPusher.setPosition(JEWELPUSHER_SERVO_STOPPED_POS);
        telemetry.log().add("Reset");
    }

    public void disablePwm(HardwareMap hardwareMap) {
        ServoController servoController = hardwareMap.servoController.get("Servo Controller 1");
        servoController.pwmDisable();
    }

    public void enablePwm(HardwareMap hardwareMap) {
        ServoController servoController = hardwareMap.servoController.get("Servo Controller 1");
        servoController.pwmEnable();
    }
}