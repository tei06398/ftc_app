package org.firstinspires.ftc.teamcode;

import android.content.SharedPreferences;
import android.preference.PreferenceManager;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "Jewel Auton Relic Recovery")
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

        gunnerFunction = new GunnerFunction(motorWinch, motorRelicSlide, servoGlyphter, servoGlyphterRotation, jewelPusher, telemetry);


        // RobotDriving instantiation
        robotDriving = new RobotDriving(motorLF, motorLB, motorRF, motorRB, telemetry, 0);
        steering = robotDriving.getSteering();
        gunnerFunction.defaultServos();

        waitForStart();

        telemetry.setAutoClear(false);

        this.jewelPusher.setPosition(JEWEL_PUSHER_DOWN);
        sleep(2000);
        double red = colorSensor.red();
        double blue = colorSensor.blue();
        boolean isRedTeam = startPosition.equals("RED_RELIC") || startPosition.equals("RED_MIDDLE");

        telemetry.addData("Red", red);
        telemetry.addData("Blue", blue);
        telemetry.addData("Is red team", isRedTeam);
        telemetry.update();

        if(red > blue) {
            if(isRedTeam){
                knockJewel(JewelPosition.RIGHT);
            }
            else{
                knockJewel(JewelPosition.LEFT);
            }
        }
        else{
            if(isRedTeam){
                knockJewel(JewelPosition.LEFT);
            }
            else{
                knockJewel(JewelPosition.RIGHT);
            }
        }


    }

    public void knockJewel(JewelPosition jewelPosition) {
        if (jewelPosition == JewelPosition.LEFT) telemetry.addData("Knocking", "left");
        if (jewelPosition == JewelPosition.RIGHT) telemetry.addData("Knocking", "right");
        telemetry.update();

        steering.setSpeedRatio(0.3);

        long startTime = System.currentTimeMillis();
        if (jewelPosition == JewelPosition.LEFT) {
            steering.turnCounterclockwise();
        } else {
            steering.turnClockwise();
        }
        steering.finishSteering();
        sleep(200);
        steering.stopAllMotors();

        this.jewelPusher.setPosition(JEWEL_PUSHER_UP);
    }

    public enum JewelPosition {
        LEFT, RIGHT
    }
}
