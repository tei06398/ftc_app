package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.util.GunnerFunction;
import org.firstinspires.ftc.teamcode.util.drive.DiagonalVectorSteeringBuilder;
import org.firstinspires.ftc.teamcode.util.drive.MecanumDriveFunction;

import static org.firstinspires.ftc.teamcode.util.deprecated.RobotDriving.MAX_SPEED_RATIO;
import static org.firstinspires.ftc.teamcode.util.deprecated.RobotDriving.MIN_SPEED_RATIO;
import static org.firstinspires.ftc.teamcode.util.deprecated.RobotDriving.NORMAL_SPEED_RATIO;

/**
 * The main teleop which is used at competitions.
 */
@TeleOp(name = "Main Teleop")
public class MainTeleop extends OpMode {
    private MecanumDriveFunction driveFunction;
    private DiagonalVectorSteeringBuilder steering;
    private GunnerFunction gunnerFunction;

    private boolean allowGamepad2B = true; // Toggles gamepad2's B key

    // The weight given to rotation (as opposed to left/right strafe) during pivoting
    private final static double BLOCK_ROTATION_WEIGHT = 0.5;

    public void init() {
        // No motors/servos are instantiated here because everything is done in the RobotDriving, GunnerFunction, etc.
        // classes.

        driveFunction = new MecanumDriveFunction(hardwareMap, telemetry, 1);

        gunnerFunction = new GunnerFunction(hardwareMap, telemetry);

        steering = new DiagonalVectorSteeringBuilder();
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

            steering.strafeRadians(angle);
        } else {
            telemetry.addData("angle: ", 0);
        }

        // Arrow keys: also driving
        if (this.gamepad1.dpad_right) {
            steering.strafeDegrees(0, MIN_SPEED_RATIO);
        }
        if (this.gamepad1.dpad_up) {
            steering.strafeDegrees(90, MIN_SPEED_RATIO);
        }
        if (this.gamepad1.dpad_left) {
            steering.strafeDegrees(180, MIN_SPEED_RATIO);
        }
        if (this.gamepad1.dpad_down) {
            steering.strafeDegrees(270, MIN_SPEED_RATIO);
        }

        // Right trigger: minimum speed
        if (this.gamepad1.right_trigger > 0.5) {
            driveFunction.setSpeedRatio(MIN_SPEED_RATIO);
        } else if (this.gamepad1.left_trigger > 0.5) {
            // Left trigger: maximum speed
            driveFunction.setSpeedRatio(MAX_SPEED_RATIO);
        } else {
            driveFunction.setSpeedRatio(NORMAL_SPEED_RATIO);
        }

        // Right bumper: move around block counterclockwise
        if (this.gamepad1.right_bumper) steering.pivot(false, BLOCK_ROTATION_WEIGHT);

        // Left bumper: move around block clockwise
        if (this.gamepad1.left_bumper) steering.pivot(true, BLOCK_ROTATION_WEIGHT);

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

        // A: expand relic slide
        // Y: retract
        if (this.gamepad2.a) {
            gunnerFunction.expandRelicSlide();
        }
        else if (this.gamepad2.y) {
            gunnerFunction.retractRelicSlide();
        }
        else {
            gunnerFunction.stopRelicSlide();
        }

        // B: toggle glyphter rotation
        if (this.gamepad2.b) {
            if (allowGamepad2B) {
                allowGamepad2B = false;
                gunnerFunction.rotateGlyphter();
            }
        } else {
            allowGamepad2B = true;
        }

        // Build steering object and put power in the motors.
        driveFunction.steer(steering);

        telemetry.update();
    }
}
