package org.firstinspires.ftc.teamcode.util;

import android.content.SharedPreferences;
import android.preference.PreferenceManager;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "Jewel Auton")
public class JewelAuton extends LinearOpMode {
    protected DcMotor motorLF = null;
    protected DcMotor motorRF = null;
    protected DcMotor motorLB = null;
    protected DcMotor motorRB = null;
    protected Servo jewelPusher = null;
    protected GunnerFunction gunnerFunction;
    protected DcMotor motorWinch = null;
    protected DcMotor motorRelicSlide = null;
    protected Servo servoGlyphter = null;
    protected Servo servoGlyphterRotation = null;

    protected RobotDriving robotDriving;
    protected RobotDriving.Steering steering;

    SharedPreferences sharedPref;
    protected String startPosition; //RED_RELIC, RED_MIDDLE, BLUE_RELIC, BLUE_MIDDLE
    protected int JEWEL_PUSHER_DOWN = 0;
    protected int JEWEL_PUSHER_UP = 100;

    protected ColorSensor colorSensor;

    @Override
    public void runOpMode() throws InterruptedException {
        //Sets up Motors
        this.motorLF = this.hardwareMap.dcMotor.get("lfMotor");
        this.motorRF = this.hardwareMap.dcMotor.get("rfMotor");
        this.motorLB = this.hardwareMap.dcMotor.get("lbMotor");
        this.motorRB = this.hardwareMap.dcMotor.get("rbMotor");

        this.motorLF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.motorRF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.motorLB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.motorRB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.colorSensor = this.hardwareMap.colorSensor.get("colorSensor");
        this.jewelPusher = this.hardwareMap.servo.get("jewelPusher");

        this.motorWinch = this.hardwareMap.dcMotor.get("winchMotor");
        this.motorRelicSlide = this.hardwareMap.dcMotor.get("relicSlideMotor");
        this.servoGlyphter = this.hardwareMap.servo.get("glyphterServo");
        this.servoGlyphterRotation = this.hardwareMap.servo.get("glyphterRotationServo");


        sharedPref = PreferenceManager.getDefaultSharedPreferences(this.hardwareMap.appContext);
        startPosition = sharedPref.getString("auton_start_position", "RED_RELIC");

        gunnerFunction = new GunnerFunction(hardwareMap, telemetry);


        // RobotDriving instantiation
        robotDriving = new RobotDriving(hardwareMap, telemetry, 1);
        steering = robotDriving.getSteering();
        gunnerFunction.reset();

        waitForStart();
        telemetry.setAutoClear(false);

        this.jewelPusher.setPosition(JEWEL_PUSHER_DOWN);

        jankySleep(2000);

        double red = colorSensor.red();
        double blue = colorSensor.blue();

        boolean isRedTeam = startPosition.equals("RED_RELIC") || startPosition.equals("RED_MIDDLE");

        telemetry.addData("Red", red);
        telemetry.addData("Blue", blue);
        telemetry.addData("Is red team", isRedTeam);
        telemetry.update();
        jankySleep(1000);

        if(red > blue) {
            if(isRedTeam){
                knockJewel(JewelPosition.LEFT);

            }
            else{
                knockJewel(JewelPosition.RIGHT);

            }
        }
        else if (red < blue){
            if(isRedTeam){
                knockJewel(JewelPosition.RIGHT);

            }
            else{
                knockJewel(JewelPosition.LEFT);

            }
        }
        else {
            this.jewelPusher.setPosition(JEWEL_PUSHER_UP);
            jankySleep(1000);
        }


        if(startPosition.equals("RED_MIDDLE")){
            steering.moveDegrees(220);
            steering.finishSteering();
            jankySleep(1000);
            steering.stopAllMotors();
        }
        else if(startPosition.equals("RED_RELIC")){
            steering.moveDegrees(160);
            steering.finishSteering();
            jankySleep(1000);
            steering.stopAllMotors();
        }
        else if(startPosition.equals("BLUE_MIDDLE")){
            motorRF.setPower(0);
            motorRB.setPower(0);
            motorLF.setPower(0);
            motorLB.setPower(0);
            motorRF.setPower(-0.5);
            motorRB.setPower(0.5);
            motorLF.setPower(-0.5);
            motorLB.setPower(0.5);
            jankySleep(1400);
            motorRF.setPower(0);
            motorRB.setPower(0);
            motorLF.setPower(0);
            motorLB.setPower(0);
            motorRF.setPower(0.5);
            motorRB.setPower(0.5);
            motorLF.setPower(-0.5);
            motorLB.setPower(-0.5);
            jankySleep(300);
            motorRF.setPower(0);
            motorRB.setPower(0);
            motorLF.setPower(0);
            motorLB.setPower(0);
        }
        else if(startPosition.equals("BLUE_RELIC")){
            steering.moveDegrees(20);
            steering.finishSteering();
            jankySleep(1000);
            steering.stopAllMotors();
        }
    }

    //Currently not working for some reason. Test at home at a later time.
    public void knockJewel(JewelPosition jewelPosition) {
        if (jewelPosition == JewelPosition.LEFT) telemetry.addData("Knocking", "left");
        if (jewelPosition == JewelPosition.RIGHT) telemetry.addData("Knocking", "right");
        telemetry.update();
        steering.setSpeedRatio(0.3);
        steering.stopAllMotors();

        //Move to knock jewel
        if (jewelPosition == JewelPosition.LEFT) steering.turnCounterclockwise();
        else steering.turnClockwise();
        jankySleep(300);
        steering.stopAllMotors();

        //Bring jewel pusher back up
        this.jewelPusher.setPosition(JEWEL_PUSHER_UP);
        jankySleep(1000);

        //Return to original position
        if (jewelPosition == JewelPosition.LEFT) steering.turnClockwise();
        else steering.turnCounterclockwise();
        jankySleep(300);
        steering.stopAllMotors();
    }

    public void jankySleep(long time) {
        int count = 0;
        double root;
        long startTime = System.currentTimeMillis();
        while (System.currentTimeMillis() - startTime < time) {
            root = Math.sqrt(Math.abs(count));
            count++;
        }

    }

    public enum JewelPosition {
        LEFT, RIGHT
    }
}
