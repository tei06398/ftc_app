package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Servo Test")
public class ServoTestMode extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Servo servo = this.hardwareMap.servo.get("digitalServo");
        DcMotor motor = this.hardwareMap.dcMotor.get("motor");
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        double position = 0;
        final double rate = 1.0 / 180;

        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.dpad_left) {
                position -= rate;
            }
            if (gamepad1.dpad_right) {
                position += rate;
            }
            if (position < 0.0) {
                position = 0;
            }
            if (position > 1.0) {
                position = 1;
            }
            if (gamepad1.a) servo.setDirection(Servo.Direction.FORWARD);
            if (gamepad1.b) servo.setDirection(Servo.Direction.REVERSE);
            if (gamepad1.x) {
                motor.setPower(.5);
            } else if (gamepad1.y) {
                motor.setPower(-.5);
            } else {
                motor.setPower(0);
            }

            servo.setPosition(position);
            telemetry.addData("Servo position", position);
            telemetry.update();
            sleep(10);
        }
    }
}