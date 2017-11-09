package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Relic Recovery Official Tele-Op Mode")
public class RRTeleOp extends OpMode {
    /* Declare here any fields you might find useful. */
    protected DcMotor motorLF = null;
    protected DcMotor motorRF = null; //declares motors
    protected DcMotor motorLB = null;
    protected DcMotor motorRB = null;

    public void loop(){

        final double MAX_SPEED_RATIO = 1; //sets the top speed for drive train

        //SET DRIVE POWER: gamepad1 right trigger
        //sets a correction factor for accuracy mode
        final double SPEED_RATIO = this.gamepad1.right_trigger > 0.5 ? 0.35 : MAX_SPEED_RATIO;

        //the base powers for all 4 motors
        double powerLF = 0;
        double powerLB = 0;
        double powerRF = 0;
        double powerRB = 0;

        //Right Stick: Controls orientation of robot
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

        //Left Stick: Controls linear movement of robot
        //if (Math.abs(this.gamepad1.left_stick_x) > 0.1 || Math.abs(this.gamepad1.left_stick_y) > 0.1) {
        double angle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x);
        // speeds for each of the axes that the robot can move
        double speed1 = Math.cos(45-angle);
        double speed2 = Math.sin(45-angle);

        // so there's going to be a speed that's always maximum
        double divider = Math.max(Math.abs(speed1), Math.abs(speed2));

        powerLF += speed1/divider;
        powerRB += -speed1/divider;
        powerLB += -speed2/divider;
        powerRF += speed2/divider;

        double maxRawPower = Math.max(Math.max(powerLF, powerLB), Math.max(powerRF, powerRB));
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
