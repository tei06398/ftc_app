package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * A utility class that controls all the gunner functions (opening and closing the glyphter, rotating the glypter, etc).
 */
public class GunnerFunction {
    private static final double GLYPHTER_SERVO_LEFT_CLOSE_POWER = 0.55;
    private static final double GLYPHTER_SERVO_LEFT_OPEN_POWER = 0.45;
    private static final double GLYPHTER_SERVO_RIGHT_CLOSE_POWER = 0.45;
    private static final double GLYPHTER_SERVO_RIGHT_OPEN_POWER = 0.55;
    private static final double SERVO_STOP_POWER = 0.5;

    private static final double GLYPHTER_ROTATION_SERVO_NORMAL_POS = 180;
    private static final double GLYPHTER_ROTATION_SERVO_ROTATED_POS = 0;
    private static final double JEWELPUSHER_SERVO_UP_POS = 100;

    private final DcMotor motorRelicSlide;
    private final DcMotor motorWinch;
    private final Servo servoGlyphterRotation;
    private final Servo servoJewelPusher;
    private final Servo servoGlyphterLeft;
    private final Servo servoGlyphterRight;
    private final Telemetry telemetry;

    // TODO: Move jewel pusher utilities to separate class
    private static final double JEWELPUSHER_SERVO_DOWN_POS = 0;

    // Assume that the glyphter is rotated at the beginning so we can fix it at the beginning of tele-op
    private boolean isGlyphterRotated = true;

    GunnerFunction(HardwareMap hardwareMap, Telemetry telemetry) {
        // Load the needed devices from the hardware map
        this.motorWinch = hardwareMap.dcMotor.get("winchMotor");
        this.motorRelicSlide = hardwareMap.dcMotor.get("relicSlideMotor");
        this.servoGlyphterLeft = hardwareMap.servo.get("glyphterServoLeft");
        this.servoGlyphterRight = hardwareMap.servo.get("glyphterServoRight");
        this.servoGlyphterRotation = hardwareMap.servo.get("glyphterRotationServo");
        this.servoJewelPusher = hardwareMap.servo.get("jewelPusher");
        this.telemetry = telemetry;
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
        servoGlyphterLeft.setPosition(GLYPHTER_SERVO_LEFT_OPEN_POWER);
        servoGlyphterRight.setPosition(GLYPHTER_SERVO_RIGHT_OPEN_POWER);
    }

    public void closeGlyphter() {
        servoGlyphterLeft.setPosition(GLYPHTER_SERVO_LEFT_CLOSE_POWER);
        servoGlyphterRight.setPosition(GLYPHTER_SERVO_RIGHT_CLOSE_POWER);
    }

    public void stopGlyphter() {
        servoGlyphterLeft.setPosition(SERVO_STOP_POWER);
        servoGlyphterRight.setPosition(SERVO_STOP_POWER);
    }

    public void expandRelicSlide() {
        motorRelicSlide.setPower(0.2);
    }

    public void retractRelicSlide() {
        motorRelicSlide.setPower(-0.2);
    }

    public void stopRelicSlide() {
        motorRelicSlide.setPower(0);
    }

    public void rotateGlyphter() {
        if(isGlyphterRotated){
            servoGlyphterRotation.setPosition(GLYPHTER_ROTATION_SERVO_NORMAL_POS);
            isGlyphterRotated = false;
        }
        else{
            servoGlyphterRotation.setPosition(GLYPHTER_ROTATION_SERVO_ROTATED_POS);
            isGlyphterRotated = true;
        }
    }

    public void reset() {
        servoGlyphterRotation.setPosition(GLYPHTER_ROTATION_SERVO_NORMAL_POS);
        closeGlyphter();
        servoJewelPusher.setPosition(JEWELPUSHER_SERVO_UP_POS);
    }
}