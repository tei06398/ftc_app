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
    protected DcMotor motorLF = null;
    protected DcMotor motorRF = null;
    protected DcMotor motorLB = null;
    protected DcMotor motorRB = null;

    protected RobotDriving robotDriving;
    protected RobotDriving.Steering steering;
    protected GunnerFunction gunnerFunction;

    private boolean allowGamepad2B = true;

    private boolean disableA1 = false;
    private boolean disableB1 = false;


    private final static double BLOCK_ROTATION_WEIGHT = 0.5; // make this final later

    public void init(){
        //Instantiates motors and servos, sets operating mode
        this.motorLF = this.hardwareMap.dcMotor.get("lfMotor");
        this.motorRF = this.hardwareMap.dcMotor.get("rfMotor");
        this.motorLB = this.hardwareMap.dcMotor.get("lbMotor");
        this.motorRB = this.hardwareMap.dcMotor.get("rbMotor");

        this.motorLF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.motorRF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.motorLB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.motorRB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robotDriving = new RobotDriving(motorLF,motorLB,motorRF,motorRB,telemetry);

        gunnerFunction = new GunnerFunction(hardwareMap, telemetry);

        steering = robotDriving.getSteering();

        gunnerFunction.reset();
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
        } else {
            // TODO: Might remove later
            gunnerFunction.stopGlyphter();
        }

        // Left trigger required for endgame functions
        if (this.gamepad2.left_trigger > 0) {
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
        }

        steering.finishSteering();

        telemetry.update();
    }
}
