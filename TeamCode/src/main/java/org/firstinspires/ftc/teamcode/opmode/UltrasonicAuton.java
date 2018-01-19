package org.firstinspires.ftc.teamcode.opmode;

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
import org.firstinspires.ftc.teamcode.util.UltrasonicFunction;
import org.firstinspires.ftc.teamcode.util.RobotDriving;

/**
 * Created 12/30/2017
 */
@Autonomous(name = "Ultrasonic Auton")
public class UltrasonicAuton extends LinearOpMode {
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
    protected int JEWEL_PUSHER_DOWN = 0;
    protected int JEWEL_PUSHER_UP = 100;

    private static final double SPEED_RATIO = 0.3;

    //Required distance from wall at the beginning
    protected int wallDistance = 30;

    private VuforiaLocalizer vuforia;

    char readVuMark(VuforiaTrackable relicTemplate) {
        RelicRecoveryVuMark vuMark = null;
        int total = 0;
        int left = 0;
        int right = 0;
        int center = 0;
        while (total < 3) {
            vuMark = RelicRecoveryVuMark.from(relicTemplate);
            if (vuMark == RelicRecoveryVuMark.LEFT) {
                left++;
                total++;
            } else if (vuMark == RelicRecoveryVuMark.RIGHT) {
                right++;
                total++;
            } else if (vuMark == RelicRecoveryVuMark.CENTER) {
                center++;
                total++;
            } else {
                telemetry.addData("unknown", "");telemetry.update();
                // unknown
                total++;
            }

        }
        if (left > right && left > center) {
            return 'l'; //Left is most likely Correct
        } else if (right > left && right > center) {
            return 'r'; //Right is most likely Correct
        } else if (center > right && center > left) {
            return 'c'; //Center is most likely Correct
        }
        return '!';
    }


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
        robotDriving = new RobotDriving(hardwareMap, telemetry, 1);
        steering = robotDriving.getSteering();
        steering.setSpeedRatio(SPEED_RATIO);

        //Ultrasonic function instantiation
        ultrasonicFunction = new UltrasonicFunction(hardwareMap, telemetry);

        //Tell Vuforia to display video feed
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        //Disable Video Feed with the following:
        //VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();


        //Set License Key, tell Vuforia to use back Camera, finalize Vuforia Parameters for Localizer
        parameters.vuforiaLicenseKey = "AUrG/9b/////AAAAGYb5udEIt0W7p8AYwNbs5lUO4Ojghb2IvJN64Q6ZnRvUfbl59Au5c48n2lykKrsfZgYx9m2HpOgdNFmLaxhilDQIc0mmohbk5IjXvKkGGJR4OiqNtqYVDncXZpb/esaPeFTLtkbJFAlEs+oPwcAKoO5FctEuFyEgz1IJc6/MRphweDiXuJ86Rqs81UVeOlNXdr3QtZazJcViHHeWkv5pJUvefJWbjXSZvOZFlITDaTbyPIibQsHsbDg+B0IBMHK+d8fs58anmJ1VJEoQ9Xo0YDpHgosgx996zfSjhAK8YcIxLQdCkFbJtQtkpAl14jz+Yz86QvP8AMtFeMRuIYrPWhQvPJ3VU/Jm1Qmz+p/jbJ8k";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);

        //Load in the VuMark dataset from assets
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);

        //Wait for OpMode Init Button to be Pressed
        waitForStart();

        //Activate the VuMark Dataset as Current Tracked Object
        relicTrackables.activate();

        /* TEST */
        telemetry.setAutoClear(false);
        telemetry.setMsTransmissionInterval(0);

        //Get a semi-reliable reading of the Pictograph
        char pictograph = 'E';
        pictograph = readVuMark(relicTemplate);

        telemetry.addData("Pictograph", "" + pictograph);
        telemetry.update();

        if (pictograph == '!') {
            telemetry.addData("Pictograph", "Unreliable");
            //Displays in the event that 3/3 times, the data returned by readVuMark() has been 1L,1C,1R, not allowing for a logical interpretation.
        } else if (pictograph == 'l') {
            telemetry.addData("Pictograph", "Left");
        } else if (pictograph == 'r') {
            telemetry.addData("Pictograph", "Right");
        } else if (pictograph == 'c') {
            telemetry.addData("Pictograph", "Center");
        } else {
            telemetry.addData("Pictograph", "ERROR");
            //Displays only if the initial value of pictograph remains unchanged, which shouldn't occur.
        }

        //Detect whiffle ball location
        telemetry.update();

        this.jewelPusher.setPosition(JEWEL_PUSHER_DOWN);

        sleep(2000);

        double red = colorSensor.red();
        double blue = colorSensor.blue();

        boolean isRedTeam = startPosition.equals("RED_RELIC") || startPosition.equals("RED_MIDDLE");

        telemetry.addData("Red", red);
        telemetry.addData("Blue", blue);
        telemetry.addData("Is red team", isRedTeam);
        telemetry.update();
        sleep(1000);

        if(red > blue) {
            if(isRedTeam)knockJewel(JewelPosition.LEFT);
            else knockJewel(JewelPosition.RIGHT);
        }
        else if (red < blue){
            if(isRedTeam) knockJewel(JewelPosition.RIGHT);
            else knockJewel(JewelPosition.LEFT);
        }
        else {
            this.jewelPusher.setPosition(JEWEL_PUSHER_UP);
            sleep(1000);
        }
        telemetry.setAutoClear(true);
        driveToCryptobox(pictograph == 'l'? CrypoboxPosition.LEFT : (pictograph == 'r' ? CrypoboxPosition.RIGHT : CrypoboxPosition.CENTER));
    }

    public void moveAlongWall(boolean moveRight, boolean senseRight, int sideDistance, int wallDistance) {
        double clockwiseTurnWeight = 0;
        double forwardWeight = 0;

        double distanceLF;
        double distanceRF;
        boolean keepMoving = true;
        while (keepMoving && opModeIsActive()) {

            //Set power in direction of motion
            if (moveRight) {
                steering.moveDegrees(0, 0.5);
            } else {
                steering.moveDegrees(180, 0.5);
            }

            //Sense distances to walls
            distanceLF = ultrasonicFunction.getLF();
            distanceRF = ultrasonicFunction.getRF();

            //Determine robot turning off course
            if (distanceLF + 1 < distanceRF) {
                clockwiseTurnWeight -= 0.01;
            } else if (distanceRF + 1 < distanceLF) {
                clockwiseTurnWeight += 0.01;
            }

            //Determine robot drifting off course
            if ((distanceLF + distanceRF) / 2 + 1 < wallDistance) {
                forwardWeight -= 0.01;
            } else if ((distanceLF + distanceRF) / 2 - 1 > wallDistance) {
                forwardWeight += 0.01;
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
    }

    public void turnNinety(boolean isClockwise) {
        if (isClockwise) {
            while (ultrasonicFunction.getLeft() > ultrasonicFunction.getRight() || ultrasonicFunction.getLF() > ultrasonicFunction.getRF()) {
                steering.turn(true);
                steering.stopAllMotors();
            }
        } else {
            while (ultrasonicFunction.getLeft() < ultrasonicFunction.getRight() || ultrasonicFunction.getLF() < ultrasonicFunction.getRF()) {
                steering.turn(false);
                steering.stopAllMotors();
            }
        }
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

    public void knockJewel(JewelPosition jewelPosition) {
        if (jewelPosition == JewelPosition.LEFT) telemetry.addData("Knocking", "left");
        if (jewelPosition == JewelPosition.RIGHT) telemetry.addData("Knocking", "right");
        telemetry.update();
        steering.setSpeedRatio(0.3);
        steering.stopAllMotors();

        //Move to knock jewel
        if (jewelPosition == JewelPosition.LEFT) steering.turnCounterclockwise();
        else steering.turnClockwise();
        sleep(300);
        steering.stopAllMotors();

        //Bring jewel pusher back up
        this.jewelPusher.setPosition(JEWEL_PUSHER_UP);
        sleep(1000);

        //Return to original position
        if (jewelPosition == JewelPosition.LEFT) steering.turnClockwise();
        else steering.turnCounterclockwise();
        sleep(300);
        steering.stopAllMotors();
    }

    public enum JewelPosition {
        LEFT, RIGHT
    }

    public enum CrypoboxPosition {
        LEFT, CENTER, RIGHT
    }
}