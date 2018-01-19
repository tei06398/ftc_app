package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.util.GunnerFunction;

/**
 * Various tests involving tele-op. Nothing meaningful here.
 */
@TeleOp(name = "Main Test")
public class MainTestMode extends OpMode {
    GunnerFunction gunnerFunction;
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
        telemetry.update();
    }

    public void init(){
        gunnerFunction = new GunnerFunction(hardwareMap, telemetry);
        gunnerFunction.raiseJewelPusher();
        pusherPos = "Up";
    }


}
