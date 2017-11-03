package org.firstinspires.ftc.teamcode;

import android.graphics.Color;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by Gabriel Kammer on 10/17/16
 */
@TeleOp(name = "Relic Recovery Official Tele-Op Mode")
public class RRTeleOp extends OpMode {
    /* Declare here any fields you might find useful. */
    protected DcMotor motorLeft = null;
    protected DcMotor motorRight = null; //declares motors
    protected DcMotor centerOmni = null;
    protected DcMotor catapult = null;
    protected DcMotor ballPicker = null;
    protected Servo buttonPusher = null;
    protected UltrasonicSensor ultrasonic = null;
    protected ColorSensor cSensor = null;
    protected Servo lSweeper = null;
    protected Servo rSweeper = null;

    public void loop(){
        //filler
    }
    public void init(){
        //filler
    }

    @Override
    public void main() throws InterruptedException {
        /* Initialize our hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names you assigned during the robot configuration
         * step you did in the FTC Robot Controller app on the phone.
         */
        this.motorLeft = this.hardwareMap.dcMotor.get("lMotor");
        this.motorRight = this.hardwareMap.dcMotor.get("rMotor"); //instantiates
        this.catapult = this.hardwareMap.dcMotor.get("catapult");
        this.ballPicker = this.hardwareMap.dcMotor.get("ballPicker");
        this.buttonPusher = this.hardwareMap.servo.get("buttonPusher");
        this.ultrasonic = this.hardwareMap.ultrasonicSensor.get("ultrasonic");
        this.cSensor = this.hardwareMap.colorSensor.get("cSensor");
        this.lSweeper = this.hardwareMap.servo.get("lSweeper");
        this.rSweeper = this.hardwareMap.servo.get("rSweeper");
        try {
            this.centerOmni = this.hardwareMap.dcMotor.get("centerOmni");
        } catch (Exception e) {}

        this.buttonPusher.setDirection(Servo.Direction.REVERSE);
        this.motorLeft.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        this.motorRight.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        this.motorLeft.setDirection(DcMotor.Direction.REVERSE);


        double driveSpeedRatio = 1; //sets the top speed for drive train
        double correctedSpeedRatio; //sets a correction factor for accuracy mode
        double catapultSpeed = 0.25; //sets top catapult speed
        double ballPickerSpeed = 0.25; //sets top ball picker speed
        double buttonPusherPosition = 1;
        double lSweeperPosition = 0;
        double rSweeperPosition = 0;

        // Wait for the game to start
        waitForStart();
        while (opModeIsActive()) {
            this.updateGamepads();  //updates game pads

            //SET DRIVE POWER: gamepad1 right trigger
            if (this.gamepad1.right_trigger > 0.5) {correctedSpeedRatio = 0.35;}
            else {correctedSpeedRatio = driveSpeedRatio;}

            //SET MOTOR POWER: gamepad1 left and right sticks
            if (this.gamepad1.left_stick_y > 0.1) {this.motorLeft.setPower(-correctedSpeedRatio);}
            else if (this.gamepad1.left_stick_y < -0.1){this.motorLeft.setPower(correctedSpeedRatio);}
            else {this.motorLeft.setPower(0);}
            if (this.gamepad1.right_stick_y > 0.1) {this.motorRight.setPower(-correctedSpeedRatio);}
            else if (this.gamepad1.right_stick_y < -0.1) {this.motorRight.setPower(correctedSpeedRatio);}
            else {this.motorRight.setPower(0);}

            //SET MIDDLE OMNI POWER: gamepad1 dpad left and right
            if (this.gamepad1.dpad_left) {centerOmni.setPower(1.0);}
            else if (this.gamepad1.dpad_right) {centerOmni.setPower(-1.0);}
            else {centerOmni.setPower(0);}

            //RUN CATAPULT: Gamepad2 right trigger and bumper
            if (this.gamepad2.right_bumper) {this.catapult.setPower(catapultSpeed);}
            else if (this.gamepad2.right_trigger > 0.5) {this.catapult.setPower(-catapultSpeed);}
            else {this.catapult.setPower(0);}

            //RUN BALL PICKER: Gamepad2 left trigger and bumper
            if (this.gamepad2.left_bumper) {
                if(lSweeperPosition > 0.4) {lSweeper.setPosition(0.4);}
                if(rSweeperPosition < 0.3) {rSweeper.setPosition(0.3);}
                this.ballPicker.setPower(ballPickerSpeed);
            } else if (this.gamepad2.left_trigger > 0.5) {
                if(lSweeperPosition > 0.4){lSweeper.setPosition(0.4);}
                if(rSweeperPosition < 0.3) {rSweeper.setPosition(0.3);}
                this.ballPicker.setPower(-ballPickerSpeed);
            } else {this.ballPicker.setPower(0);}

            //RUN BUTTON PUSHER: Gamepad2 dpad up and down
            if (gamepad2.dpad_up) {
                buttonPusherPosition = Math.max(0, buttonPusherPosition - 0.05);
                buttonPusher.setPosition(buttonPusherPosition);
            } else if (gamepad2.dpad_down) {
                buttonPusherPosition = Math.min(1, buttonPusherPosition + 0.05);
                buttonPusher.setPosition(buttonPusherPosition);
            }

            //RUN SWEEPERS: Gamepad2 left stick up and down
            if (gamepad2.left_stick_y > 0.5 && !this.gamepad2.left_bumper && this.gamepad2.left_trigger <= 0.5) {
                lSweeperPosition = Math.min(.84, lSweeperPosition + 0.05);
                rSweeperPosition = -83*lSweeperPosition/79 + 1663/1580;
                lSweeper.setPosition(Math.min(lSweeperPosition, 0.55));
                rSweeper.setPosition(rSweeperPosition);
            } else if (gamepad2.left_stick_y < -0.5 && !this.gamepad2.left_bumper && this.gamepad2.left_trigger <= 0.5) {
                lSweeperPosition = Math.max(0.05, lSweeperPosition - 0.05);
                rSweeperPosition = -83*lSweeperPosition/79 + 1663/1580;
                lSweeper.setPosition(Math.min(lSweeperPosition, 0.55));
                rSweeper.setPosition(rSweeperPosition);
            }

            //CODE FOR SMALL NUDGE MOVEMENTS:
            if (this.gamepad1.right_bumper) {
                this.motorLeft.setPower(driveSpeedRatio);
                this.motorRight.setPower(-driveSpeedRatio);
                Thread.sleep(100);
                this.motorLeft.setPower(0);
                this.motorRight.setPower(0);
            }
            if (this.gamepad1.left_bumper) {
                this.motorLeft.setPower(-driveSpeedRatio);
                this.motorRight.setPower(driveSpeedRatio);
                Thread.sleep(100);
                this.motorLeft.setPower(0);
                this.motorRight.setPower(0);
            }

            telemetry.addData("Left Sweeper: ", lSweeperPosition);
            telemetry.addData("Right Sweeper: ", rSweeperPosition);
            telemetry.addData("Red: ", cSensor.red());
            telemetry.addData("Green: ", cSensor.green());
            telemetry.addData("Blue: ", cSensor.blue());
            telemetry.addData("Ultrasonic: ", ultrasonic.getUltrasonicLevel());
            telemetry.update();
        }
    }
}
