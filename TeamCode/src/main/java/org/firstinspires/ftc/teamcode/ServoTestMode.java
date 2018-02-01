package org.firstinspires.ftc.teamcode;

import android.util.Log;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;

@TeleOp(name="Servo Test")
public class ServoTestMode extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        //Servo servo = this.hardwareMap.servo.get("digitalServo");
        //ServoController servoController = this.hardwareMap.servoController.get("Servo Controller 1");this.hardwareMap.logDevices();
        //servoController.pwmEnable();
        DcMotor motor = this.hardwareMap.dcMotor.get("motor");
        Servo servo1 = this.hardwareMap.servo.get("digitalServo1");
        Servo servo2 = this.hardwareMap.servo.get("digitalServo2");
        //ServoImplEx servoImplEx = (ServoImplEx) servo;
        //Log.i("NiskyRobot", "{Log is here}");
        //this.hardwareMap.logDevices();

        //PWMOutput pwmOutput = hardwareMap.pwmOutput.get("digitalServo");


        //Increase Close and Lift
        //Grabber servo1
        //Lifter servo2

        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        double lifterposition = 0;
        double grabberposition = 0;
        final double rate = 1.0 / 180;

        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.dpad_left) {
                lifterposition -= rate;
            }
            if (gamepad1.dpad_right) {
                lifterposition += rate;
            }
            if (gamepad1.dpad_up) {
                grabberposition += rate;
            }
            if (gamepad1.dpad_down) {
                grabberposition -= rate;
            }
            if (lifterposition < 0.0) {
                lifterposition = 0;
            }
            if (lifterposition > 1.0) {
                lifterposition = 1;
            }
            if (grabberposition < 0.0) {
                grabberposition = 0;
            }
            if (grabberposition > 1.0) {
                grabberposition = 1;
            }
            //if (gamepad1.a) pwmOutput.setPulseWidthOutputTime(1000);
            //if (gamepad2.b) pwmOutput.setPulseWidthOutputTime(2000);
            if (gamepad1.x) {
                motor.setPower(.5);
            } else if (gamepad1.y) {
                motor.setPower(-.5);
            } else {
                motor.setPower(0);
            }

            servo1.setPosition(lifterposition);
            servo2.setPosition(grabberposition);
            telemetry.addData("Grabber position", grabberposition);
            telemetry.addData("Lifter position", lifterposition);
            telemetry.update();
            sleep(10);
        }
    }
}
