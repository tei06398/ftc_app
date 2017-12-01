package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class GunnerFunction {
    private final DcMotor motorRelicSlide;
    private final DcMotor winchMotor;
    private final Servo glyphterServo;
    private final Servo jewelServo;
    private final Telemetry telemetry;

    // put in actual values later
    private static final int GLYPHTER_SERVO_CLOSE_POS = 0;
    private static final int GLYPHTER_SERVO_OPEN_POS = 100;

    GunnerFunction(DcMotor winch, DcMotor motorRelicSlide, Servo glyphter, Servo jewel, Telemetry t) {
        winchMotor = winch;
        this.motorRelicSlide = motorRelicSlide;
        glyphterServo = glyphter;
        jewelServo = jewel;
        telemetry = t;
    }

    public void upWinch() {
        winchMotor.setPower(0.5);
    }

    public void downWinch() {
        winchMotor.setPower(-0.5);
    }

    public void stopWinch() {
        winchMotor.setPower(0);
    }

    public void openGlyphter() {
        glyphterServo.setPosition(GLYPHTER_SERVO_OPEN_POS);
    }

    public void closeGlyphter() {
        glyphterServo.setPosition(GLYPHTER_SERVO_CLOSE_POS);
    }

    public void expandRelicSlide() {
        motorRelicSlide.setPower(0.5);
    }

    public void retractRelicSlide() {
        motorRelicSlide.setPower(-0.5);
    }
}
