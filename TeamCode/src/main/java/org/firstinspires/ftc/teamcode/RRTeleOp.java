package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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
        if (this.gamepad1.retractRelicSlide) {
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
        // Right Stick: Turn/Rotate
        if (this.gamepad1.right_stick_x > 0.1) {
            steering.turnCounterclockwise();
        } else if (this.gamepad1.right_stick_x < -0.1) {
            steering.turnClockwise();
        }

        // Left Stick: Driving
        if (Math.abs(this.gamepad1.left_stick_x) > 0.1 || Math.abs(this.gamepad1.left_stick_y) > 0.1) {
            double angle = Math.atan2(gamepad1.left_stick_y, -gamepad1.left_stick_x);
            telemetry.addData("angle: ", angle);

            steering.moveRadians(-angle);
        } else {
            telemetry.addData("angle: ", 0);
        }

        // Arrow Keys: Compass Rose Drive
        if (this.gamepad1.dpad_right) {
            //Turns out that around point code moves straighter than original code here
            //steering.moveDegrees(180, MIN_SPEED_RATIO);
            steering.aroundPoint(false, BLOCK_ROTATION_WEIGHT);
        }
        if (this.gamepad1.dpad_up) {
            steering.moveDegrees(90, MIN_SPEED_RATIO);
        }
        if (this.gamepad1.dpad_left) {
            //Turns out that around point code moves straighter than original code here
            //steering.moveDegrees(0, MIN_SPEED_RATIO);
            steering.aroundPoint(true, BLOCK_ROTATION_WEIGHT);

        }
        if (this.gamepad1.dpad_down) {
            steering.moveDegrees(270, MIN_SPEED_RATIO);
        }

        if (this.gamepad1.right_trigger > 0.5) {
            // Right Trigger: Minimum Speed Ratio
            steering.setSpeedRatio(MIN_SPEED_RATIO);
        } else if (this.gamepad1.left_trigger > 0.5) {
            // Left Trigger: Maximum Speed
            steering.setSpeedRatio(MAX_SPEED_RATIO);
        } else {
            steering.setSpeedRatio(NORMAL_SPEED_RATIO);
        }

        // Right Bumper: Move Around Block CCW
        if (this.gamepad1.right_bumper) steering.aroundPoint(false, BLOCK_ROTATION_WEIGHT);

        // Left Bumper: Move Around Block CW
        if (this.gamepad1.left_bumper) steering.aroundPoint(true, BLOCK_ROTATION_WEIGHT);

        // GAMEPAD 2 (GUNNER)
        if (this.gamepad2.dpad_up) {
            // DPad Up: Raise Glyphter
            gunnerFunction.upWinch();
        } else if (this.gamepad2.dpad_down) {
            // DPad Down: Lower Glyphter
            gunnerFunction.downWinch();
        } else {
            gunnerFunction.stopWinch();
        }

        if (this.gamepad2.y) {
            // Y Button: Extend Relic Slide
            gunnerFunction.extendRelicSlide();
        } else if (this.gamepad2.a) {
            // A Button: Retract Relic Slide
            gunnerFunction.retractRelicSlide();
        } else {
            gunnerFunction.stopRelicSlide();
        }

        if (this.gamepad2.left_bumper) {
            //Left Bumper: Open Glyphter
            gunnerFunction.openGlyphter();
        } else if (this.gamepad2.left_trigger>0.4) {
            //Left Trigger: Close Glyphter
            gunnerFunction.closeGlyphter();
        }

        if (this.gamepad2.right_bumper) {
            //Right Bumper: Open Grabber
            gunnerFunction.openRelicGrabberIncremental();
        } else if (this.gamepad2.right_trigger>0.4) {
            //Right Trigger: Close Grabber
            gunnerFunction.closeRelicGrabberIncremental();
        }

        if (this.gamepad2.b) {
            gunnerFunction.lowerRelicLifter();
        } else if (this.gamepad2.x) {
            gunnerFunction.raiseRelicLifter();
        }

        // Finish Steering, putting Power into Hardware
        steering.finishSteering();

        telemetry.update();
    }
}
