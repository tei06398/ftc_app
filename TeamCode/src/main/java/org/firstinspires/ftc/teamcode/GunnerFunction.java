package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class GunnerFunction {
    private DcMotor winchMotor;
    private Servo glyphterServo;
    private Servo jewelServo;
    private Telemetry telemetry;

    GunnerFunction(DcMotor winch, Servo glyphter, Servo jewel, Telemetry t) {
        winchMotor = winch;
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
}
