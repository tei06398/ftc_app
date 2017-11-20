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
    // Declares motors
    protected DcMotor motorLF = null;
    protected DcMotor motorRF = null;
    protected DcMotor motorLB = null;
    protected DcMotor motorRB = null;
    protected DcMotor motorWinch = null;
    protected Servo servoGlyphter = null;
    protected Servo servoJewelTipper = null;
    protected RobotDriving robotDriving;
    protected RobotDriving.Steering steering;
    protected GunnerFunction gunnerFunction;

    public void loop() {
        steering.setSpeedRatio((this.gamepad1.right_trigger > 0.5) ? MIN_SPEED_RATIO : MAX_SPEED_RATIO);

        //Controls orientation of robot
        if (this.gamepad1.right_stick_x > 0.1) {
            steering.turnClockwise();
        } else if (this.gamepad1.right_stick_x < -0.1) {
            steering.turnCounterclockwise();
        }

        if (this.gamepad2.dpad_up) {
            gunnerFunction.upWinch();
        } else if (this.gamepad2.dpad_down) {
            gunnerFunction.downWinch();
        } else {
            gunnerFunction.stopWinch();
        }

        //Controls linear movement of robot
        // Only actually move if the joystick is offset.
        if (Math.abs(this.gamepad1.left_stick_x) > 0.1 || Math.abs(this.gamepad1.left_stick_y) > 0.1) {
            double angle = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x);
            telemetry.addData("angle: ", angle);
            
            steering.moveRadians(angle);
        } else {
            telemetry.addData("angle: ", 0);
        }
        
        steering.finishSteering();

        telemetry.addData("Right stick x: ", this.gamepad1.right_stick_x);
        telemetry.addData("Left stick x: ", this.gamepad1.left_stick_x);
        telemetry.addData("Left stick y: ", this.gamepad1.left_stick_y);
        /*telemetry.addData("powerLF: ", powerLF);
        telemetry.addData("powerRB: ", powerRB);
        telemetry.addData("powerLB: ", powerLB);
        telemetry.addData("powerRF: ", powerRF);*/
        telemetry.update();
    }

    public void init(){

        //Instantiates motors and servos, sets operating mode
        this.motorLF = this.hardwareMap.dcMotor.get("lfMotor");
        this.motorRF = this.hardwareMap.dcMotor.get("rfMotor");
        this.motorLB = this.hardwareMap.dcMotor.get("lbMotor");
        this.motorRB = this.hardwareMap.dcMotor.get("rbMotor");
        this.motorWinch = this.hardwareMap.dcMotor.get("winchMotor");
        this.servoGlyphter = this.hardwareMap.servo.get("glyphterServo");
        this.servoJewelTipper = this.hardwareMap.servo.get("jewelTipperServo");
        this.motorLF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.motorRF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.motorLB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.motorRB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
        robotDriving = new RobotDriving(motorLF,motorLB,motorRF,motorRB,telemetry);
        gunnerFunction = new GunnerFunction(motorWinch, servoGlyphter, servoJewelTipper, telemetry);
        
        steering = robotDriving.getSteering();
    }
}
