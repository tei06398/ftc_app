package org.firstinspires.ftc.teamcode;

import android.content.SharedPreferences;
import android.preference.PreferenceManager;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * Created 12/30/2017
 */
@Autonomous(name = "RR Ultrasonic Auton Mode Test")
public class UltrasonicAutonTest extends LinearOpMode {
    //Declares Motors
    protected DcMotor motorLF = null;
    protected DcMotor motorRF = null;
    protected DcMotor motorLB = null;
    protected DcMotor motorRB = null;
    protected UltrasonicSensor ultrasonicLeft = null;
    protected UltrasonicSensor ultrasonicRight = null;
    protected UltrasonicSensor ultrasonicRF = null;
    protected UltrasonicSensor ultrasonicLF = null;
    protected Servo jewelPusher = null;

    protected RobotDriving robotDriving;
    protected RobotDriving.Steering steering;

    protected UltrasonicFunction ultrasonicFunction;
    protected ColorSensor colorSensor;

    SharedPreferences sharedPref;
    protected String startPosition; //RED_RELIC, RED_MIDDLE, BLUE_RELIC, BLUE_MIDDLE

    private static final double SPEED_RATIO = 0.2;

    //Required distance from wall at the beginning
    protected int wallDistance = 30;

    @Override
    public void runOpMode() throws InterruptedException {
        //Sets up Motors
        this.motorLF = this.hardwareMap.dcMotor.get("lfMotor");
        this.motorRF = this.hardwareMap.dcMotor.get("rfMotor");
        this.motorLB = this.hardwareMap.dcMotor.get("lbMotor");
        this.motorRB = this.hardwareMap.dcMotor.get("rbMotor");
        this.ultrasonicLeft = this.hardwareMap.ultrasonicSensor.get("ultrasonicLeft"); //module 2, port 1
        this.ultrasonicRight = this.hardwareMap.ultrasonicSensor.get("ultrasonicRight");//module 2, port 2
        this.ultrasonicLF = this.hardwareMap.ultrasonicSensor.get("ultrasonicLF"); //module 3, port 3
        this.ultrasonicRF = this.hardwareMap.ultrasonicSensor.get("ultrasonicRF"); //module 4, port 4
        this.colorSensor = this.hardwareMap.colorSensor.get("colorSensor");

        this.motorLF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.motorRF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.motorLB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.motorRB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        this.jewelPusher = this.hardwareMap.servo.get("jewelPusher");

        sharedPref = PreferenceManager.getDefaultSharedPreferences(this.hardwareMap.appContext);
        startPosition = sharedPref.getString("auton_start_position", "RED_RELIC");

        // RobotDriving instantiation
        robotDriving = new RobotDriving(motorLF, motorLB, motorRF, motorRB, telemetry, 1);
        steering = robotDriving.getSteering();
        steering.setSpeedRatio(SPEED_RATIO);

        //Ultrasonic function instantiation
        ultrasonicFunction = new UltrasonicFunction(ultrasonicLeft, ultrasonicRight, ultrasonicRF, ultrasonicLF, telemetry);

        waitForStart();

        telemetry.setAutoClear(true);

        moveAlongWall(false, true, 160, 50);
        telemetry.addData("Now starting alignment process: ", "true");
        telemetry.update();
        sleep(1000);
        alignToWall();
        telemetry.addData("Now starting movement along wall: ", "true");
        telemetry.update();
        sleep(1000);
        moveAlongWall(false, false, 50, 50);
        telemetry.addData("Now starting turn process: ", "true");
        telemetry.update();
        sleep(1000);
        turnNinety(false);
        //driveToCryptobox(pictograph == 'l'? CrypoboxPosition.LEFT : (pictograph == 'r' ? CrypoboxPosition.RIGHT : CrypoboxPosition.CENTER));
    }

    public void moveAlongWall(boolean moveRight, boolean senseRight, int sideDistance, int wallDistance) {
        double clockwiseTurnWeight = 0;
        double forwardWeight = 0;
        final double MAX_TURN_WEIGHT = 0.2;
        final double MAX_FORWARD_WEIGHT = 0.4;
        double distanceLF;
        double distanceRF;
        boolean keepMoving = true;
        if (moveRight && senseRight) ultrasonicFunction.setRight(255);
        else if (moveRight && !senseRight) ultrasonicFunction.setLeft(0);
        else if (!moveRight && senseRight) ultrasonicFunction.setRight(0);
        else if (!moveRight && !senseRight) ultrasonicFunction.setLeft(255);
        while (keepMoving && opModeIsActive()) {
            //Set power in direction of motion
            if (moveRight) {
                steering.moveDegrees(0, 1);
            } else {
                steering.moveDegrees(180, 1);
            }

            //Sense distances to walls
            distanceLF = ultrasonicFunction.getLF();
            distanceRF = ultrasonicFunction.getRF();

            //Determine robot turning off course
            if (distanceLF + 1 < distanceRF) {
                clockwiseTurnWeight = Math.max(clockwiseTurnWeight - 0.005, -MAX_TURN_WEIGHT);
            } else if (distanceRF + 1 < distanceLF) {
                clockwiseTurnWeight = Math.min(clockwiseTurnWeight + 0.005, MAX_TURN_WEIGHT);
            }

            //Determine robot drifting off course
            if ((distanceLF + distanceRF) / 2 + 1 < wallDistance) {
                forwardWeight = Math.max(forwardWeight - 0.005, -MAX_FORWARD_WEIGHT);
            } else if ((distanceLF + distanceRF) / 2 - 1 > wallDistance) {
                forwardWeight = Math.min(forwardWeight + 0.005, MAX_FORWARD_WEIGHT);
            }

            telemetry.addData("Forward weight", forwardWeight);
            telemetry.addData("Clockwise turn weight", clockwiseTurnWeight);
            telemetry.addData("Code RF", distanceRF);
            telemetry.addData("Code LF", distanceLF);
            ultrasonicFunction.printTestData();
            telemetry.update();

            if (forwardWeight > 0) {
                steering.moveDegrees(90, forwardWeight);
            } else {
                steering.moveDegrees(270, -forwardWeight);
            }

            //Weird issue with left/right mirroring, that's why clockwiseTurnWeight is negated
            steering.turn(clockwiseTurnWeight);

            steering.finishSteering();

            //determine whether to keep moving
            if (moveRight && senseRight) {
                keepMoving = ultrasonicFunction.getRight() > sideDistance;
            } else if (!moveRight && senseRight) {
                keepMoving = ultrasonicFunction.getRight() < sideDistance;
            } else if (moveRight) {
                keepMoving = ultrasonicFunction.getLeft() < sideDistance;
            } else {
                keepMoving = ultrasonicFunction.getLeft() > sideDistance;
            }
        }
        steering.stopAllMotors();
    }

    public void turnNinety(boolean isClockwise) {
        if (isClockwise) {
            //leftDist is the distance detected from ultrasonicLeft in the previous tick
            double leftDist = 255;
            while (ultrasonicFunction.getLeft() <= leftDist) {
                steering.turn(1);
                steering.finishSteering();
                leftDist = ultrasonicFunction.getLeft();
            }
            steering.stopAllMotors();
            alignToWall();
        } else {
            //rightDist is the distance detected from ultrasonicLeft in the previous tick
            double rightDist = 255;
            while (ultrasonicFunction.getLeft() <= rightDist) {
                steering.turn(1);
                steering.finishSteering();
                rightDist = ultrasonicFunction.getRight();
            }
            steering.stopAllMotors();
            alignToWall();
        }
    }

    public void alignToWall() {
        while (Math.abs(ultrasonicFunction.getLF() - ultrasonicFunction.getRF()) > 1) {
            if (ultrasonicFunction.getLF() + 1 < ultrasonicFunction.getRF()) {
                steering.turn(-1);
            } else if (ultrasonicFunction.getRF() + 1 < ultrasonicFunction.getLF()) {
                steering.turn(1);
            } else {
                steering.stopAllMotors();
                break;
            }
            steering.finishSteering();
        }
    }

    public void approachCryptobox() {

    }

    public void driveToCryptobox(CrypoboxPosition crypoboxPosition) {
        if (startPosition.equals("RED_RELIC")) {
            moveAlongWall(false, true, 150, 50);
        } else if (startPosition.equals("RED_MIDDLE")) {
            moveAlongWall(false, false, 60, 50);
        } else if (startPosition.equals("BLUE_RELIC")) {
            moveAlongWall(true, false, 150, 50);
        } else {
            moveAlongWall(true, true, 60, 50);
        }
    }

    public enum CrypoboxPosition {
        LEFT, CENTER, RIGHT
    }
}