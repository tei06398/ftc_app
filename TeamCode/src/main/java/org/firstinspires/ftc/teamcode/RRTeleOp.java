package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import static org.firstinspires.ftc.teamcode.RobotDriving.MAX_SPEED_RATIO;
import static org.firstinspires.ftc.teamcode.RobotDriving.MIN_SPEED_RATIO;
import static org.firstinspires.ftc.teamcode.RobotDriving.NORMAL_SPEED_RATIO;

@TeleOp(name = "Relic Recovery Official Tele-Op Mode")
public class RRTeleOp extends OpMode {
    protected RobotDriving robotDriving;
    protected RobotDriving.Steering steering;
    protected GunnerFunction gunnerFunction;

    // The weight given to rotation (as opposed to left/right strafe) during pivoting
    private final static double BLOCK_ROTATION_WEIGHT = 0.5;

    private boolean disable2A = false;
    private boolean disable2B = false;

    public void init() {
        // No motors/servos are instantiated here because everything is done in the RobotDriving, GunnerFunction, etc.
        // classes.

        robotDriving = new RobotDriving(hardwareMap, telemetry);

        gunnerFunction = new GunnerFunction(hardwareMap, telemetry);

        steering = robotDriving.getSteering();
    }

    public void loop() {
        // TESTING
        /*
        if (this.gamepad1.a) {
            if (!disableA1) BLOCK_ROTATION_WEIGHT += 0.05;
            disableA1 = true;
        } else {
            disableA1 = false;
        }
        if (this.gamepad1.b) {
            if (!disableB1) BLOCK_ROTATION_WEIGHT -= 0.05;
            disableB1 = true;
        } else {
            disableB1 = false;
        }
        telemetry.addData("block rotation weight: ", BLOCK_ROTATION_WEIGHT);*/

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
        if (this.gamepad1.dpad_right) {
            steering.moveDegrees(0, MIN_SPEED_RATIO);
        }
        if (this.gamepad1.dpad_up) {
            steering.moveDegrees(90, MIN_SPEED_RATIO);
        }
        if (this.gamepad1.dpad_left) {
            steering.moveDegrees(180, MIN_SPEED_RATIO);
        }
        if (this.gamepad1.dpad_down) {
            steering.moveDegrees(270, MIN_SPEED_RATIO);
        }

        // Right trigger: minimum speed
        if (this.gamepad1.right_trigger > 0.5) {
            steering.setSpeedRatio(MIN_SPEED_RATIO);
        } else if (this.gamepad1.left_trigger > 0.5) {
            // Left trigger: maximum speed
            steering.setSpeedRatio(MAX_SPEED_RATIO);
        } else {
            steering.setSpeedRatio(NORMAL_SPEED_RATIO);
        }

        // Right bumper: move around block counterclockwise
        if (this.gamepad1.right_bumper) steering.aroundPoint(false, BLOCK_ROTATION_WEIGHT);

        // Left bumper: move around block clockwise
        if (this.gamepad1.left_bumper) steering.aroundPoint(true, BLOCK_ROTATION_WEIGHT);

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
        if (this.gamepad2.left_bumper) {
            gunnerFunction.closeGlyphter();
        } else if (this.gamepad2.right_bumper) {
            gunnerFunction.openGlyphter();
        }

        // Left trigger: close glyphter incrementally
        // Right trigger: open glyphter incrementally
        if (this.gamepad2.left_trigger > 0) {
            gunnerFunction.closeGlyphterIncremental();
        } else if (this.gamepad2.right_trigger > 0) {
            gunnerFunction.openGlyphterIncremental();
        }

        // X: expand relic slide
        // Y: retract
        if (this.gamepad2.x) {
            gunnerFunction.expandRelicSlide();
        }
        else if (this.gamepad2.y) {
            gunnerFunction.retractRelicSlide();
        }
        else {
            gunnerFunction.stopRelicSlide();
        }

        // A: toggle relic grabber pivot
        if (gamepad2.a) {
            if (!disable2A) {
                gunnerFunction.toggleRelicGrabberPivot();
            }
            disable2A = true;
        } else {
            disable2A = false;
        }

        // B: toggle relic grabber
        if (gamepad2.b) {
            if (!disable2B) {
                gunnerFunction.toggleRelicGrabber();
            }
            disable2B = true;
        } else {
            disable2B = false;
        }

        // Finish the steering, which puts power in the motors.
        steering.finishSteering();

        telemetry.update();
    }
}
