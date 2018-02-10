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
    private final static double BLOCK_ROTATION_WEIGHT = 0.6;
    private final static double TURNING_SPEED_BOOST = 0.3;

    private boolean disable2A = false;
    private boolean disable2B = false;
    private boolean extendRelicSlide = true;
    private long extendRelicSlideStartTime = 0;

    public void init() {
        // No motors/servos are instantiated here because everything is done in the RobotDriving, GunnerFunction, etc.
        // classes.

        robotDriving = new RobotDriving(hardwareMap, telemetry);

        gunnerFunction = new GunnerFunction(hardwareMap, telemetry);

        steering = robotDriving.getSteering();

        gunnerFunction.stopAutonGlyphter();
    }

    public void loop() {
        gunnerFunction.stopAutonGlyphter();
        if (extendRelicSlide) {
            if (extendRelicSlideStartTime == 0) {
                extendRelicSlideStartTime = System.currentTimeMillis();
                gunnerFunction.extendRelicSlide();
            } else if (System.currentTimeMillis() - extendRelicSlideStartTime > 200) {
                gunnerFunction.stopRelicSlide();
                extendRelicSlide = false;
            }
        }
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

        // Arrow Keys: Compass Rose Drive
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

        //Set Speed Ratio depending on the triggers pressed
        if (this.gamepad1.right_trigger > 0.5) {
            // Left Trigger: Minimum Speed Ratio
            steering.setSpeedRatio(MAX_SPEED_RATIO);
        } else if (this.gamepad1.left_trigger > 0.5) {
            // Right Trigger: Maximum Speed
            steering.setSpeedRatio(MIN_SPEED_RATIO);
        } else {
            steering.setSpeedRatio(NORMAL_SPEED_RATIO);
        }

        // Right Stick: Turn/Rotate
        if (this.gamepad1.right_stick_x > 0.1) {
            steering.turnClockwise();
            steering.setSpeedRatio(Math.min(1, steering.getSpeedRatio() + TURNING_SPEED_BOOST));
        } else if (this.gamepad1.right_stick_x < -0.1) {
            steering.turnCounterclockwise();
            steering.setSpeedRatio(Math.min(1, steering.getSpeedRatio() + TURNING_SPEED_BOOST));
        }

        // Left Stick: Driving
        if (Math.abs(this.gamepad1.left_stick_x) > 0.1 || Math.abs(this.gamepad1.left_stick_y) > 0.1) {
            double angle = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x);
            telemetry.addData("angle: ", angle);

            steering.moveRadians(angle);
        } else {
            telemetry.addData("angle: ", 0);
        }

        // Right Bumper: Move Around Block CCW
        if (this.gamepad1.right_bumper) steering.aroundPoint(false, BLOCK_ROTATION_WEIGHT);

        // Left Bumper: Move Around Block CW
        if (this.gamepad1.left_bumper) steering.aroundPoint(true, BLOCK_ROTATION_WEIGHT);

        // GAMEPAD 2 (GUNNER)
        if (this.gamepad2.dpad_up) {
            gunnerFunction.extendRelicSlide();
        } else if (this.gamepad2.dpad_down) {
            gunnerFunction.retractRelicSlide();
        } else if (!extendRelicSlide){
            gunnerFunction.stopRelicSlide();
        }

        if (this.gamepad2.y) {
            gunnerFunction.upWinch();
        } else if (this.gamepad2.a) {
            gunnerFunction.downWinch();
        } else {
            gunnerFunction.stopWinch();
        }

        if (this.gamepad2.right_bumper) {
            gunnerFunction.openGlyphter();
        } else if (this.gamepad2.right_trigger>0.4) {
            gunnerFunction.closeGlyphter();
        }

        if (this.gamepad2.left_bumper) {
            gunnerFunction.openRelicGrabberIncremental();
        } else if (this.gamepad2.left_trigger>0.4) {
            gunnerFunction.closeRelicGrabberIncremental();
        }

        if (this.gamepad2.dpad_right) {
            gunnerFunction.lowerRelicLifter();
        } else if (this.gamepad2.dpad_left) {
            gunnerFunction.raiseRelicLifter();
        }

        if (this.gamepad2.left_stick_y > 0.1) {
            gunnerFunction.releaseRelic();
        } else if (this.gamepad2.left_stick_y < -0.1) {
            gunnerFunction.grabRelic();
        }

        // Finish Steering, putting Power into Hardware
        steering.finishSteering();

        telemetry.update();
    }
}
