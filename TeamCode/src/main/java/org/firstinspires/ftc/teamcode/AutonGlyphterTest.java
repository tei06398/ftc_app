package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Auton Glyphter Test")
public class AutonGlyphterTest extends OpMode {
    TwoStateServo autonGlyphter;
    Servo jewelPusher;
    @Override
    public void init() {
        autonGlyphter = new TwoStateServo(hardwareMap.servo.get("autonGlyphter"), 0, 1);
        jewelPusher = hardwareMap.servo.get("jewelPusher");
        jewelPusher.setPosition(0.5);
    }

    @Override
    public void loop() {
        if (this.gamepad1.dpad_up) {
            autonGlyphter.active();
        } else if (this.gamepad1.dpad_down) {
            autonGlyphter.passive();
        } else {
            autonGlyphter.getServo().setPosition(0.5);
        }
    }
}
