package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name = "Trigger Tester")
public class TriggerTester extends OpMode {
    @Override
    public void init() {

    }

    @Override
    public void loop() {
        telemetry.addData("GP1 LT", this.gamepad1.left_trigger);
        telemetry.addData("GP1 RT", this.gamepad1.right_trigger);
        telemetry.addData("GP2 LT", this.gamepad2.left_trigger);
        telemetry.addData("GP2 RT", this.gamepad2.right_trigger);
        telemetry.update();
    }
}
