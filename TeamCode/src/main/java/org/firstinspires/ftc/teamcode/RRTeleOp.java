package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import static org.firstinspires.ftc.teamcode.RobotDriving.MAX_SPEED_RATIO;
import static org.firstinspires.ftc.teamcode.RobotDriving.MIN_SPEED_RATIO;

/**
 * Last Modified 11/7/2017
 */
@TeleOp(name = "Relic Recovery Official Tele-Op Mode")
public class RRTeleOp extends OpMode {
    /* Declare here any fields you might find useful. */
    protected DcMotor motorLF = null;
    // Declares motors
    protected DcMotor motorRF = null;
    protected DcMotor motorLB = null;
    protected DcMotor motorRB = null;
    protected DcMotor motorWinch = null;
    protected DcMotor motorRelicSlide = null;
    protected Servo servoGlyphter = null;
    protected Servo servoGlyphterRotation = null;
    protected RobotDriving robotDriving;
    protected RobotDriving.Steering steering;
    protected GunnerFunction gunnerFunction;
    private static double BLOCK_ROTATION_WEIGHT = 0.5; // make this final later

    public void loop() {
        steering.setSpeedRatio((this.gamepad1.right_trigger > 0.5) ? MIN_SPEED_RATIO : MAX_SPEED_RATIO);

        // TESTING
        if (this.gamepad1.a) BLOCK_ROTATION_WEIGHT += 0.05;
        if (this.gamepad1.b) BLOCK_ROTATION_WEIGHT -= 0.05;
        telemetry.addData("block rotation weight: ", BLOCK_ROTATION_WEIGHT);

        // GAMEPAD 1 (DRIVER)
        // Right stick: turn
        if (this.gamepad1.right_stick_x > 0.1) {
            steering.turnClockwise();
        } else if (this.gamepad1.right_stick_x < -0.1) {
            steering.turnCounterclockwise();
        }

        // Left stick: driving
        if (Math.abs(this.gamepad1.left_stick_x) > 0.1 || Math.abs(this.gamepad1.left_stick_y) > 0.1) {
            double angle = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x);
            telemetry.addData("angle: ", angle);

            steering.moveRadians(angle);
        } else {
            telemetry.addData("angle: ", 0);
        }

        // Arrow keys: also driving
        if (this.gamepad1.dpad_right) steering.moveDegrees(0);
        if (this.gamepad1.dpad_up) steering.moveDegrees(90);
        if (this.gamepad1.dpad_left) steering.moveDegrees(180);
        if (this.gamepad1.dpad_down) steering.moveDegrees(270);

        // Left bumper: move around block counterclockwise
        if (this.gamepad1.left_bumper) steering.aroundPoint(false, BLOCK_ROTATION_WEIGHT);

        // Right bumper: clockwise
        if (this.gamepad1.right_bumper) steering.aroundPoint(true, BLOCK_ROTATION_WEIGHT);

        // GAMEPAD 2 (GUNNER)
        // Up/down keys: winch
        if (this.gamepad2.dpad_up) {
            gunnerFunction.upWinch();
        } else if (this.gamepad2.dpad_down) {
            gunnerFunction.downWinch();
        } else {
            gunnerFunction.stopWinch();
        }

        // Left bumper: close glyphter
        // Right bumper: open glyphter
        if (this.gamepad2.left_bumper) gunnerFunction.closeGlyphter();
        if (this.gamepad2.right_bumper) gunnerFunction.openGlyphter();

        // Left trigger required for endgame functions
        if (this.gamepad2.left_trigger > 0) {
            // Y: expand relic slide
            // X: retract
            if (this.gamepad2.y) {
                gunnerFunction.expandRelicSlide();
            }
            else if (this.gamepad2.x) {
                gunnerFunction.retractRelicSlide();
            }
            else {
                gunnerFunction.stopRelicSlide();
            }

            // B: toggle glyphter rotation
            if (this.gamepad2.b) {
                gunnerFunction.rotateGlyphter();
            }
        }

        steering.finishSteering();

        telemetry.addData("Right stick x: ", this.gamepad1.right_stick_x);
        telemetry.addData("Left stick x: ", this.gamepad1.left_stick_x);
        telemetry.addData("Left stick y: ", this.gamepad1.left_stick_y);
        telemetry.update();
    }

    public void init(){
        //Instantiates motors and servos, sets operating mode
        this.motorLF = this.hardwareMap.dcMotor.get("lfMotor");
        this.motorRF = this.hardwareMap.dcMotor.get("rfMotor");
        this.motorLB = this.hardwareMap.dcMotor.get("lbMotor");
        this.motorRB = this.hardwareMap.dcMotor.get("rbMotor");
        this.motorWinch = this.hardwareMap.dcMotor.get("winchMotor");
        this.motorRelicSlide = this.hardwareMap.dcMotor.get("relicSlideMotor");
        this.servoGlyphter = this.hardwareMap.servo.get("glyphterServo");
        this.servoGlyphterRotation = this.hardwareMap.servo.get("glyphterRotationServo");
        this.motorLF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.motorRF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.motorLB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.motorRB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
        robotDriving = new RobotDriving(motorLF,motorLB,motorRF,motorRB,telemetry);
        gunnerFunction = new GunnerFunction(motorWinch, motorRelicSlide, servoGlyphter, servoGlyphterRotation, telemetry);
        
        steering = robotDriving.getSteering();

        // Snap the glyphter rotation servo into the correct spot
        gunnerFunction.rotateGlyphter();
    }
}
