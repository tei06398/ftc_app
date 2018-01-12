package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Various tests involving tele-op. Nothing meaningful here.
 */
@TeleOp(name = "Test Mode")
public class TestTeleOp extends OpMode {
    /* Declare here any fields you might find useful. */
    // Declares motors
    protected DcMotor motorLF = null;
    protected DcMotor motorRF = null;
    protected DcMotor motorLB = null;
    protected DcMotor motorRB = null;
    protected Servo glyphterServoLeft = null;
    protected Servo glyphterServoRight = null;

    GunnerFunction gunnerFunction = new GunnerFunction(hardwareMap, telemetry);

    private double position = 100;
    private double position2 = 100;

    public void loop(){
        if (this.gamepad1.y) position += 0.1;
        if (this.gamepad1.a) position -= 0.1;
        if (this.gamepad1.x) position2 += 0.1;
        if (this.gamepad1.b) position2 -= 0.1;
        if (this.gamepad1.left_bumper) {gunnerFunction.lowerJewelPusher();}
        if (this.gamepad1.right_bumper) {gunnerFunction.raiseJewelPusher();}

        this.glyphterServoLeft.setPosition(position);
        this.glyphterServoRight.setPosition(position2);

        telemetry.addData("glyphterServoLeft", position);
        telemetry.addData("glyphterServoRight", position2);
        telemetry.addData("glyphterServoLeft reading", glyphterServoLeft.getPosition());
        telemetry.addData("glyphterServoRight reading", glyphterServoRight.getPosition());

        telemetry.update();
    }

    public void init(){
        //Instantiates motors and servos, sets operating mode
        this.motorLF = this.hardwareMap.dcMotor.get("lfMotor");
        this.motorRF = this.hardwareMap.dcMotor.get("rfMotor");
        this.motorLB = this.hardwareMap.dcMotor.get("lbMotor");
        this.motorRB = this.hardwareMap.dcMotor.get("rbMotor");
        this.glyphterServoLeft = this.hardwareMap.servo.get("glyphterServoLeft");
        this.glyphterServoRight = this.hardwareMap.servo.get("glyphterServoRight");
        this.motorLF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.motorRF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.motorLB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.motorRB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}
