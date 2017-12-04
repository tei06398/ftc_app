package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class GunnerFunction {
    private final DcMotor motorRelicSlide;
    private final DcMotor motorWinch;
    private final Servo servoGlyphter;
    private final Servo servoGlyphterRotation;
    private final Telemetry telemetry;

    // put in actual values later
    private static final int GLYPHTER_SERVO_CLOSE_POS = 0;
    private static final int GLYPHTER_SERVO_OPEN_POS = 100;
    private static final double GLYPHTER_ROTATION_SERVO_NORMAL_POS = 0.47;
    private static final double GLYPHTER_ROTATION_SERVO_ROTATED_POS = 0.53;
    private boolean isGlyphterRotated = false;

    GunnerFunction(DcMotor motorWinch, DcMotor motorRelicSlide, Servo servoGlyphter, Servo servoGlyphterRotation, Telemetry telemetry) {
        this.motorWinch = motorWinch;
        this.motorRelicSlide = motorRelicSlide;
        this.servoGlyphter = servoGlyphter;
        this.servoGlyphterRotation = servoGlyphterRotation;
        this.telemetry = telemetry;
    }

    public void upWinch() {
        motorWinch.setPower(0.5);
    }

    public void downWinch() {
        motorWinch.setPower(-0.5);
    }

    public void stopWinch() {
        motorWinch.setPower(0);
    }

    public void openGlyphter() {
        servoGlyphter.setPosition(GLYPHTER_SERVO_OPEN_POS);
    }

    public void closeGlyphter() {
        servoGlyphter.setPosition(GLYPHTER_SERVO_CLOSE_POS);
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

/// changed Servo to CRServo, different methods to be called
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
}
