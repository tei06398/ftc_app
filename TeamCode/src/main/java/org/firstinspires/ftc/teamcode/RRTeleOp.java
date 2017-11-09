package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

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

    public void loop(){
        //filler

        final double MAX_SPEED_RATIO = 1; //sets the top speed for drive train

        //SET SPEED RATIO: gamepad1 right trigger
        //for accuracy mode, don't drive the robot at max power.
        final double SPEED_RATIO = (this.gamepad1.right_trigger > 0.5) ? 0.35 : MAX_SPEED_RATIO;

        // the base powers for all 4 motors. Power is added to these base powers,
        // then everything is scaled so that the maximum power is equal to SPEED_RATIO.
        // Note that both the orientation and movement each add a maximum of 1 to each motor's power -- 1 is sort of used as a unit.
        double powerLF = 0;
        double powerLB = 0;
        double powerRF = 0;
        double powerRB = 0;

        //Controls orientation of robot
        if (this.gamepad1.right_stick_x > 0.1) {
            powerLF += 1;
            powerLB += 1;
            powerRF += 1;
            powerRB += 1;
        } else if (this.gamepad1.right_stick_x < -0.1) {
            powerLF -= 1;
            powerLB -= 1;
            powerRF -= 1;
            powerRB -= 1;
        }

        //Controls linear movement of robot
        // Only actually move if the joystick is offset.
        if (Math.abs(this.gamepad1.left_stick_x) > 0.1 || Math.abs(this.gamepad1.left_stick_y) > 0.1) {
            double angle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x);
            // speeds for each of the axes that the robot can move
            double speed1 = Math.cos(45-angle);
            double speed2 = Math.sin(45-angle);

            // so there's always going to be a speed that's 1
            double divider = Math.max(Math.abs(speed1), Math.abs(speed2));

            powerLF += speed1 / divider;
            powerRB += -speed1 / divider;
            powerLB += -speed2 / divider;
            powerRF += speed2 / divider;
        }
        
        // The maximum base power.
        double maxRawPower = Math.max(Math.max(powerLF, powerLB), Math.max(powerRF, powerRB));
        
        // Now, actually set the powers for the motors. Dividing by maxRawPower makes the maximum power 1, and multiplying by SPEED_RATIO
        // makes the maximum power SPEED_RATIO.
        if (maxRawPower != 0) {
            this.motorLF.setPower(powerLF / maxRawPower * SPEED_RATIO);
            this.motorLB.setPower(powerLB / maxRawPower * SPEED_RATIO);
            this.motorRF.setPower(powerRF / maxRawPower * SPEED_RATIO);
            this.motorRB.setPower(powerRB / maxRawPower * SPEED_RATIO);
        }

        telemetry.update();
    }

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


    }


}
