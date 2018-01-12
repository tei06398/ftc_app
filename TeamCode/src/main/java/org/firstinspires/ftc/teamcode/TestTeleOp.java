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
    private GunnerFunction gunnerFunction;

    public void loop(){
        if (this.gamepad1.left_bumper) {gunnerFunction.lowerJewelPusher();}
        if (this.gamepad1.right_bumper) {gunnerFunction.raiseJewelPusher();}
    }

    public void init(){
        gunnerFunction = new GunnerFunction(hardwareMap, telemetry);
    }
}
