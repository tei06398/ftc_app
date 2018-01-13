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
    GunnerFunction gunnerFunction = new GunnerFunction(hardwareMap, telemetry);
    String pusherPos;
    public void loop(){
        if (this.gamepad1.dpad_down) {
            gunnerFunction.lowerJewelPusher();
            pusherPos = "Down";
        }
        if (this.gamepad1.dpad_up) {
            gunnerFunction.raiseJewelPusher();
            pusherPos = "Up";
        }
        telemetry.addData("Pusher Position: ", pusherPos);
    }

    public void init(){
        gunnerFunction.raiseJewelPusher();
        pusherPos = "Up";
    }
}
