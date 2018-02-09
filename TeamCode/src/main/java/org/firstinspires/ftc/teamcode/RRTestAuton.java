package org.firstinspires.ftc.teamcode;

import android.content.SharedPreferences;
import android.preference.PreferenceManager;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.*;

import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.*;

/**
 * The official autonomous mode.
 */
@Autonomous(name = "RR Test Auton Mode")
public class RRTestAuton extends LinearOpMode {
    protected ColorSensor jewelTipper;

    protected RobotDriving robotDriving;
    protected RobotDriving.Steering steering
            ;

    protected UltrasonicFunction ultrasonicFunction;
    protected GunnerFunction gunnerFunction;
    protected RobotLog log;
    //protected OpticalDistanceSensor leftEOPD;
    //protected OpticalDistanceSensor rightEOPD;

    SharedPreferences sharedPref;
    protected String startPosition; // RED_RELIC, RED_MIDDLE, BLUE_RELIC, BLUE_MIDDLE

    //Required distance from wall at the beginning
    /*protected int wallDistance = 30;
    protected double clockwiseTurnWeight = 0;
    protected double forwardWeight = 0;*/

    protected final double MOVE_SPEED_RATIO = 0.3;
    protected final double PRECISE_TURN_SPEED_RATIO = 0.3;
    protected final double FAST_TURN_SPEED_RATIO = 0.6;
    protected final int JEWELPUSHER_EXTENSION_TIME = 3500;
    protected final int JEWEL_PUSH_TIME = 400;

    //protected VideoCapture camera = null;

    private VuforiaLocalizer vuforia;

    @Override
    public void runOpMode() {
        this.jewelTipper = this.hardwareMap.colorSensor.get("jewelTipper");

        sharedPref = PreferenceManager.getDefaultSharedPreferences(this.hardwareMap.appContext);
        startPosition = sharedPref.getString("auton_start_position", "RED_RELIC");

        //leftEOPD = hardwareMap.opticalDistanceSensor.get("leftEOPD");
        //rightEOPD = hardwareMap.opticalDistanceSensor.get("rightEOPD");

        robotDriving = new RobotDriving(hardwareMap, telemetry);
        steering = robotDriving.getSteering();

        log = RobotLog.getRootInstance(telemetry);
        ultrasonicFunction = new UltrasonicFunction(hardwareMap, log);
        gunnerFunction = new GunnerFunction(hardwareMap, telemetry);

        /* VUFORIA CODE */

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

        steering.setSpeedRatio(MOVE_SPEED_RATIO);

        //Activate the VuMark Dataset as Current Tracked Object
        relicTrackables.activate();

        telemetry.setMsTransmissionInterval(0);

        /* GET PICTOGRAPH */

        char pictograph = readVuMark(relicTemplate);

        telemetry.addData("Pictograph", "" + pictograph);
        telemetry.update();

        if (pictograph == '!') {
            telemetry.addData("Pictograph", "Unreliable");
            //Displays in the event that 3/3 times, the data returned by readVuMark() has been 1L,1C,1R, not allowing for retractRelicSlide logical interpretation.
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
        telemetry.update();
        sleep(1000);

        while (true) {
            if (gamepad1.dpad_left) {
                turnNinety(false);
            }
            if (gamepad1.dpad_right) {
                turnNinety(true);
            }
            if (gamepad1.dpad_up) {
                alignToWall();
            }
            if (gamepad1.dpad_down) {
                break;
            }
        }
    }

    /**
     * Read the pictograph.
     * @return A character representing whether the pictograph is left, right, or center.
     */
    char readVuMark(VuforiaTrackable relicTemplate) {
        RelicRecoveryVuMark vuMark = null;
        int total = 0;
        int left = 0;
        int right = 0;
        int center = 0;
        while (total < 3) {
            vuMark = RelicRecoveryVuMark.from(relicTemplate);
            sleep(100);
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
                telemetry.addData("unknown", "");
                telemetry.update();
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

    public void moveAlongWall(boolean senseRight, int targetSideDistance, int targetWallDistance, int tolerance) {
        double sensorDistanceLF;
        double sensorDistanceRF;
        double sensorDistanceSide;
        double robotClockwiseRotation;
        double frontDistance;
        double sideDistance;
        double robotMovementAngle;
        double robotMovementDistance;
        double clockwiseTurnSpeed;
        double moveSpeed;

        boolean keepMoving = true;
        while (keepMoving && opModeIsActive() && !isStopRequested()) {
            sensorDistanceLF = ultrasonicFunction.getLF();
            sensorDistanceRF = ultrasonicFunction.getRF();
            if (senseRight) { sensorDistanceSide = ultrasonicFunction.getRight(); }
            else { sensorDistanceSide = ultrasonicFunction.getLeft(); }

            robotClockwiseRotation = Math.atan2(sensorDistanceLF - sensorDistanceRF, 20);
            frontDistance = (sensorDistanceLF + sensorDistanceRF) * Math.cos(robotClockwiseRotation) / 2;
            sideDistance = sensorDistanceSide * Math.cos(robotClockwiseRotation);

            if (senseRight) {
                robotMovementAngle = Math.atan2(frontDistance - targetWallDistance, sideDistance - targetSideDistance);
            }
            else {
                robotMovementAngle = Math.atan2(frontDistance - targetWallDistance, targetSideDistance - sideDistance);
            }

            robotMovementDistance = Math.sqrt(Math.pow(frontDistance - targetWallDistance, 2) + Math.pow(sideDistance - targetSideDistance, 2));
            clockwiseTurnSpeed = -robotClockwiseRotation / (5 * Math.sqrt(Math.toRadians(20) + Math.pow(robotClockwiseRotation, 2)));
            moveSpeed = robotMovementDistance / (3 * Math.sqrt(30 + Math.pow(robotMovementDistance, 2)));
            steering.setSpeedRatio(Math.sqrt(Math.pow(clockwiseTurnSpeed, 2) + Math.pow(moveSpeed, 2)));
            steering.turn(clockwiseTurnSpeed);
            steering.moveRadians(robotMovementAngle + robotClockwiseRotation, moveSpeed);
            steering.finishSteering();

            keepMoving = Math.abs(sensorDistanceLF - targetWallDistance) > tolerance || Math.abs(sensorDistanceRF - targetWallDistance) > tolerance || Math.abs(sensorDistanceSide - targetSideDistance) > tolerance;
            telemetry.addData("sensorDistanceLF", sensorDistanceLF);
            telemetry.addData("sensorDistanceRF", sensorDistanceRF);
            telemetry.addData("sensorDistanceSide", sensorDistanceSide);
            telemetry.addData("robotClockwiseRotation", robotClockwiseRotation);
            telemetry.addData("frontDistance", frontDistance);
            telemetry.addData("sideDistance", sideDistance);
            telemetry.addData("robotMovementAngle", robotMovementAngle);
            telemetry.addData("robotMovementDistance", robotMovementDistance);
            telemetry.addData("clockwiseTurnSpeed", clockwiseTurnSpeed);
            telemetry.addData("moveSpeed", moveSpeed);
            telemetry.update();
        }
        steering.stopAllMotors();
    }

    /**
     * Turn ninety degrees.
     */
    public void turnNinety(boolean isClockwise) {
        steering.setSpeedRatio(FAST_TURN_SPEED_RATIO);
        if (isClockwise) {
            //leftDist is the distance detected from ultrasonicLeft in the previous tick
            double leftDist = 255;

            //Turn until left distance begins to increase (meaning that robot has passed the position that it should reach)
            while (ultrasonicFunction.getLeft() <= leftDist) {
                steering.turnClockwise();
                steering.finishSteering();
                leftDist = ultrasonicFunction.getLeft();
            }
            steering.stopAllMotors();
            //Return to position of minimum left distance
            while (ultrasonicFunction.getLeft() > leftDist) {
                steering.turnCounterclockwise();
                steering.finishSteering();
                leftDist = ultrasonicFunction.getLeft();
            }
            alignToWall();
        } else {
            //rightDist is the distance detected from ultrasonicRight in the previous tick
            double rightDist = 255;

            //Turn until right distance begins to increase (meaning that robot has passed the position that it should reach)
            while (ultrasonicFunction.getRight() <= rightDist) {
                steering.turnCounterclockwise();
                steering.finishSteering();
                rightDist = ultrasonicFunction.getRight();
            }
            steering.stopAllMotors();
            //Return to position of minimum right distance
            while (ultrasonicFunction.getRight() > rightDist) {
                steering.turnClockwise();
                steering.finishSteering();
                rightDist = ultrasonicFunction.getRight();
            }
            alignToWall();
        }
    }

    /**
     * Align to retractRelicSlide wall.
     */
    public void alignToWall() {
        steering.setSpeedRatio(PRECISE_TURN_SPEED_RATIO);
        double lfDist = ultrasonicFunction.getLF();
        double rfDist = ultrasonicFunction.getRF();
        while (Math.abs(lfDist - rfDist) >= 1) {
            if (lfDist < rfDist) { steering.turnClockwise(); }
            else if (rfDist < lfDist) { steering.turnCounterclockwise(); }
            steering.finishSteering();
            lfDist = ultrasonicFunction.getLF();
            rfDist = ultrasonicFunction.getRF();
        }
        steering.stopAllMotors();
    }

    /**
     * Knock the correct jewel down.
     */
    public void knockJewel() {
        telemetry.addData("knockJewel Method called", "");
        telemetry.update();
        steering.setSpeedRatio(PRECISE_TURN_SPEED_RATIO);
        gunnerFunction.extendJewelPusher();
        sleep(JEWELPUSHER_EXTENSION_TIME);
        gunnerFunction.stopJewelPusher();
        double red = 0;
        double blue = 0;
        for (int i = 0; i < 5; i++) {
            red += jewelTipper.red();
            blue += jewelTipper.blue();
            sleep(100);
        }
        telemetry.addData("red", red);
        telemetry.addData("blue", blue);
        telemetry.update();
        boolean isRedTeam = startPosition.equals("RED_RELIC") || startPosition.equals("RED_MIDDLE");

        if(red > blue) {
            if(isRedTeam) {
                pushJewel(JewelPosition.RIGHT);
            }
            else {
                pushJewel(JewelPosition.LEFT);
            }
        }
        else if (red < blue){
            if(isRedTeam){
                pushJewel(JewelPosition.LEFT);
            }
            else{
                pushJewel(JewelPosition.RIGHT);
            }
        } else {
            gunnerFunction.retractJewelPusher();
            sleep(JEWELPUSHER_EXTENSION_TIME);
            gunnerFunction.stopJewelPusher();
        }
    }

    /**
     * Push either the left or right jewel down.
     */
    public void pushJewel(JewelPosition jewelPosition) {
        if (jewelPosition == JewelPosition.LEFT) {
            steering.turnCounterclockwise(1);
            steering.move(270, 0.5);
            steering.finishSteering();
            sleep(JEWEL_PUSH_TIME);
            steering.stopAllMotors();
            gunnerFunction.retractJewelPusher();
            sleep(JEWELPUSHER_EXTENSION_TIME);
            gunnerFunction.stopJewelPusher();
            steering.turnClockwise(1);
            steering.move(270, 0.3);
            steering.finishSteering();
            sleep(JEWEL_PUSH_TIME + 250);
            steering.stopAllMotors();
            sleep(1000);
        } else {
            steering.turnClockwise();
            steering.finishSteering();
            sleep(JEWEL_PUSH_TIME);
            steering.stopAllMotors();
            gunnerFunction.retractJewelPusher();
            sleep(JEWELPUSHER_EXTENSION_TIME);
            gunnerFunction.stopJewelPusher();
        }
    }

    public void moveTime(double degrees, long time) {
        steering.moveDegrees(degrees);
        steering.finishSteering();
        sleep(time);
        steering.stopAllMotors();
    }

    public void turnTime(boolean isClockwise, long time) {
        steering.turn(isClockwise);
        steering.finishSteering();
        sleep(time);
        steering.stopAllMotors();
    }

    public enum JewelPosition {
        LEFT, RIGHT
    }
}