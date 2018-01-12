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
    private Servo jewelPusher;
    double jewelPusherPosition;

    public void loop(){
        if (this.gamepad1.left_bumper) {
            jewelPusherPosition += 0.01;
        }
        if (this.gamepad1.right_bumper) {
            jewelPusherPosition -= 0.01;
        }
        jewelPusher.setPosition(jewelPusherPosition);
        telemetry.addData("Pos", jewelPusherPosition);
        telemetry.update();
    }

    public void init(){
        jewelPusher = this.hardwareMap.servo.get("jewelPusher");
        jewelPusherPosition = 0;
    }
}
