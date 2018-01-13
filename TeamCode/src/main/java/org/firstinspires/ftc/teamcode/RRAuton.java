package org.firstinspires.ftc.teamcode;

import android.content.SharedPreferences;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.graphics.Color;
import android.hardware.Camera;
import android.preference.Preference;
import android.preference.PreferenceManager;
import com.qualcomm.ftccommon.FtcRobotControllerService;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.FtcRobotControllerServiceState;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.*;

import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;
import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.*;

import java.io.ByteArrayInputStream;
import java.io.File;
import java.nio.ByteBuffer;

/**
 * The official autonomous mode.
 */
@Autonomous(name = "RR Official Auton Mode")
public class RRAuton extends LinearOpMode {
    protected ColorSensor colorSensor;

    protected RobotDriving robotDriving;
    protected RobotDriving.Steering steering;

    protected UltrasonicFunction ultrasonicFunction;
    protected GunnerFunction gunnerFunction;

    SharedPreferences sharedPref;
    protected String startPosition; // RED_RELIC, RED_MIDDLE, BLUE_RELIC, BLUE_MIDDLE

    //Required distance from wall at the beginning
    /*protected int wallDistance = 30;
    protected double clockwiseTurnWeight = 0;
    protected double forwardWeight = 0;*/

    protected final double MOVE_SPEED_RATIO = 0.2;
    protected final double TURN_SPEED_RATIO = 0.1;
    protected char pictograph;

    //protected VideoCapture camera = null;

    private VuforiaLocalizer vuforia;

    @Override
    public void runOpMode() {
        this.colorSensor = this.hardwareMap.colorSensor.get("colorSensor");

        sharedPref = PreferenceManager.getDefaultSharedPreferences(this.hardwareMap.appContext);
        startPosition = sharedPref.getString("auton_start_position", "RED_RELIC");

        robotDriving = new RobotDriving(hardwareMap, telemetry, 1); // higher smoothness = faster changes
        steering = robotDriving.getSteering();

        ultrasonicFunction = new UltrasonicFunction(hardwareMap, telemetry);
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


        //Activate the VuMark Dataset as Current Tracked Object
        relicTrackables.activate();

        //Wait for OpMode Init Button to be Pressed
        waitForStart();


        telemetry.setMsTransmissionInterval(0);

        /* GET PICTOGRAPH */

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

        /* KNOCK JEWEL */

        steering.setSpeedRatio(MOVE_SPEED_RATIO);

        // Lift
        gunnerFunction.upWinch();
        sleep(5000);
        gunnerFunction.stopWinch();

        // Knock jewel
        knockJewel();

        // Lower
        gunnerFunction.downWinch();
        sleep(1150);
        gunnerFunction.stopWinch();

        /* DRIVE TO CRYPTOBOX */

        checkActive();
        steering.moveDegrees(270);
        steering.finishSteering();
        sleep(500);
        steering.stopAllMotors();
        checkActive();
        gunnerFunction.upWinch();
        sleep(100);
        checkActive();
        gunnerFunction.stopWinch();
        if (startPosition.equals("RED_MIDDLE")) {
            int sideDistance;
            if (pictograph == 'l') {
                sideDistance = 86;
            } else if (pictograph == 'r') {
                sideDistance = 46;
            } else {
                sideDistance = 66;
            }
            checkActive();
            moveAlongWall(false, false, 45, 50);
            sleep(100);
            checkActive();
            turnNinety(false);
            sleep(100);
            checkActive();
            moveAlongWall(false, true, sideDistance, 50);
            sleep(100);
            checkActive();
            approachCryptobox(true, sideDistance);


        } else if (startPosition.equals("RED_RELIC")) {
            int sideDistance;
            if (pictograph == 'l') {
                sideDistance = 150;
            } else if (pictograph == 'r') {
                sideDistance = 110;
            } else {
                sideDistance = 130;
            }
            checkActive();
            moveAlongWall(false, true, sideDistance, 50);
            sleep(100);
            checkActive();
            approachCryptobox(true, sideDistance);


        } else if (startPosition.equals("BLUE_MIDDLE")) {
            int sideDistance;
            if (pictograph == 'l') {
                sideDistance = 50;
            } else if (pictograph == 'r') {
                sideDistance = 90;
            } else {
                sideDistance = 70;
            }
            checkActive();
            moveAlongWall(true, true, 45, 50);
            sleep(100);
            checkActive();
            turnNinety(true);
            sleep(100);
            checkActive();
            moveAlongWall(true, false, sideDistance, 50);
            sleep(100);
            checkActive();
            approachCryptobox(false, sideDistance);


        } else {
            int sideDistance;
            if (pictograph == 'l') {
                sideDistance = 113;
            } else if (pictograph == 'r') {
                sideDistance = 152;
            } else {
                sideDistance = 132;
            }
            checkActive();
            moveAlongWall(true, false, sideDistance, 50);
            sleep(100);
            checkActive();
            approachCryptobox(false, sideDistance);
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
        while (total < 9) {
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
        } else if (right == left && right == center) {
            telemetry.addData("Error:", "Math Bork - Pictograph Values Equal");
        }
        return '!';
    }

    /**
     * Move along a wall.
     */
    public void moveAlongWall(boolean moveRight, boolean senseRight, int sideDistance, int wallDistance) {
        steering.setSpeedRatio(MOVE_SPEED_RATIO);
        double clockwiseTurnWeight = 0;
        double forwardWeight = 0;
        final double MAX_TURN_WEIGHT = 0.1;
        final double MAX_FORWARD_WEIGHT = 0.1;

        //Rate of acceleration
        final double INCREASE_RATE = 0.005;

        //Rate of deceleration
        final double NORMALIZE_RATE = 0.02;
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
                if (clockwiseTurnWeight > 0) {
                    clockwiseTurnWeight = Math.max(clockwiseTurnWeight - NORMALIZE_RATE, -MAX_TURN_WEIGHT);
                } else {
                    clockwiseTurnWeight = Math.max(clockwiseTurnWeight - INCREASE_RATE, -MAX_TURN_WEIGHT);
                }

            } else if (distanceRF + 1 < distanceLF) {
                if (clockwiseTurnWeight > 0) {
                    clockwiseTurnWeight = Math.min(clockwiseTurnWeight + INCREASE_RATE, MAX_TURN_WEIGHT);
                } else {
                    clockwiseTurnWeight = Math.min(clockwiseTurnWeight + NORMALIZE_RATE, MAX_TURN_WEIGHT);
                }
            }

            //Determine robot drifting off course
            if ((distanceLF + distanceRF) / 2 + 1 < wallDistance) {
                if (forwardWeight > 0) {
                    forwardWeight = Math.max(forwardWeight - NORMALIZE_RATE, -MAX_FORWARD_WEIGHT);
                } else {
                    forwardWeight = Math.max(forwardWeight - INCREASE_RATE, -MAX_FORWARD_WEIGHT);
                }
            } else if ((distanceLF + distanceRF) / 2 - 1 > wallDistance) {
                if (forwardWeight > 0) {
                    forwardWeight = Math.min(forwardWeight + INCREASE_RATE, MAX_FORWARD_WEIGHT);
                } else {
                    forwardWeight = Math.min(forwardWeight + NORMALIZE_RATE, MAX_FORWARD_WEIGHT);
                }
            }

            telemetry.addData("Forward weight", forwardWeight);
            telemetry.addData("Clockwise turn weight", clockwiseTurnWeight);
            telemetry.addData("Code RF", distanceRF);
            telemetry.addData("Code LF", distanceLF);
            telemetry.addData("Pictograph", "" + pictograph);
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
        steering.stopAllMotors();
    }

    /**
     * Turn ninety degrees.
     */
    public void turnNinety(boolean isClockwise) {
        steering.setSpeedRatio(TURN_SPEED_RATIO);
        if (isClockwise) {
            //leftDist is the distance detected from ultrasonicLeft in the previous tick
            double leftDist = 255;
            steering.turn(1);
            steering.finishSteering();
            sleep(1000);
            steering.stopAllMotors();
            //Turn until left distance begins to increase (meaning that robot has passed the position that it should reach)
            while (ultrasonicFunction.getLeft() <= leftDist && opModeIsActive()) {
                steering.turn(1);
                steering.finishSteering();
                leftDist = ultrasonicFunction.getLeft();
            }
            steering.stopAllMotors();
            //Return to position of minimum left distance
            while (ultrasonicFunction.getLeft() > leftDist && opModeIsActive()) {
                steering.turn(-1);
                steering.finishSteering();
                leftDist = ultrasonicFunction.getLeft();
            }
            alignToWall();
        } else {
            //rightDist is the distance detected from ultrasonicRight in the previous tick
            double rightDist = 255;
            steering.turn(-1);
            steering.finishSteering();
            sleep(1000);
            steering.stopAllMotors();
            //Turn until right distance begins to increase (meaning that robot has passed the position that it should reach)
            while (ultrasonicFunction.getRight() <= rightDist && opModeIsActive()) {
                steering.turn(-1);
                steering.finishSteering();
                rightDist = ultrasonicFunction.getRight();
            }
            steering.stopAllMotors();
            //Return to position of minimum right distance
            while (ultrasonicFunction.getRight() > rightDist && opModeIsActive()) {
                steering.turn(1);
                steering.finishSteering();
                rightDist = ultrasonicFunction.getRight();
            }
            alignToWall();
        }
    }

    /**
     * Align to a wall.
     */
    public void alignToWall() {
        steering.setSpeedRatio(TURN_SPEED_RATIO);
        while (Math.abs(ultrasonicFunction.getLF() - ultrasonicFunction.getRF()) >= 1 && opModeIsActive()) {
            if (ultrasonicFunction.getLF() < ultrasonicFunction.getRF()) {
                steering.turn(-1);
            } else if (ultrasonicFunction.getRF() < ultrasonicFunction.getLF()) {
                steering.turn(1);
            }
            steering.finishSteering();
            if (ultrasonicFunction.getLF() == ultrasonicFunction.getRF()) {
                steering.stopAllMotors();
                break;
            }
        }
    }

    /**
     * Get closer to the crytobox once the robot is in front of it.
     */
    public void approachCryptobox(boolean senseRight, int goalSideDistance) {
        double clockwiseTurnWeight = 0;
        double rightWeight = 0;
        final double MAX_TURN_WEIGHT = 0.2;
        final double MAX_FORWARD_WEIGHT = 0.2;

        //Rate of acceleration
        final double INCREASE_RATE = 0.005;

        //Rate of deceleration
        final double NORMALIZE_RATE = 0.02;
        double sideDistance;
        double distanceLF;
        double distanceRF;
        boolean keepMoving = true;
        final int wallDistance = 35;

        steering.setSpeedRatio(MOVE_SPEED_RATIO);
        while (ultrasonicFunction.getRF() + ultrasonicFunction.getLF() > wallDistance * 2 && opModeIsActive()) {
            telemetry.addData("getRF: ", ultrasonicFunction.getRF());
            telemetry.addData("getLF: ", ultrasonicFunction.getLF());
            telemetry.update();
            distanceLF = ultrasonicFunction.getLF();
            distanceRF = ultrasonicFunction.getRF();
            sideDistance = senseRight ? ultrasonicFunction.getRight() : ultrasonicFunction.getLeft();
            if (distanceLF > distanceRF + 1) {
                if (clockwiseTurnWeight < 0) {
                    clockwiseTurnWeight = Math.min(clockwiseTurnWeight + NORMALIZE_RATE, MAX_TURN_WEIGHT);
                } else {
                    clockwiseTurnWeight = Math.min(clockwiseTurnWeight + INCREASE_RATE, MAX_TURN_WEIGHT);
                }
            }
            else if (distanceRF > distanceLF + 1) {
                if (clockwiseTurnWeight > 0) {
                    clockwiseTurnWeight = Math.max(clockwiseTurnWeight - NORMALIZE_RATE, -MAX_TURN_WEIGHT);
                } else {
                    clockwiseTurnWeight = Math.max(clockwiseTurnWeight - INCREASE_RATE, -MAX_TURN_WEIGHT);
                }
            }

            if (sideDistance > goalSideDistance + 1) {
                if (senseRight) {
                    if (rightWeight < 0) {
                        rightWeight = Math.min(rightWeight + NORMALIZE_RATE, MAX_TURN_WEIGHT);
                    } else {
                        rightWeight = Math.min(rightWeight + INCREASE_RATE, MAX_TURN_WEIGHT);
                    }
                } else {
                    if (rightWeight > 0) {
                        rightWeight = Math.max(rightWeight - NORMALIZE_RATE, -MAX_TURN_WEIGHT);
                    } else {
                        rightWeight = Math.max(rightWeight - INCREASE_RATE, -MAX_TURN_WEIGHT);
                    }
                }
            }
            else if (sideDistance < goalSideDistance - 1) {
                if (!senseRight) {
                    if (rightWeight < 0) {
                        rightWeight = Math.min(rightWeight + NORMALIZE_RATE, MAX_TURN_WEIGHT);
                    } else {
                        rightWeight = Math.min(rightWeight + INCREASE_RATE, MAX_TURN_WEIGHT);
                    }
                } else {
                    if (rightWeight > 0) {
                        rightWeight = Math.max(rightWeight - NORMALIZE_RATE, -MAX_TURN_WEIGHT);
                    } else {
                        rightWeight = Math.max(rightWeight - INCREASE_RATE, -MAX_TURN_WEIGHT);
                    }
                }
            }
            steering.moveDegrees(90, 1);
            steering.turn(clockwiseTurnWeight);
            steering.moveDegrees(0, rightWeight);
            steering.finishSteering();
        }
        steering.stopAllMotors();
        long startTime = System.currentTimeMillis();
        while (System.currentTimeMillis() - startTime < 5000 && opModeIsActive()) {
            steering.moveDegrees(90, 0.2);
            steering.moveDegrees(0, Math.sin(Math.PI*(System.currentTimeMillis()-startTime)/500));
            steering.finishSteering();
        }
        steering.stopAllMotors();
        gunnerFunction.openGlyphter();
        steering.moveDegrees(270);
        steering.finishSteering();
        sleep(1000);
        steering.stopAllMotors();
    }

    /*public void driveToCryptobox(CrypoboxPosition crypoboxPosition) {
        if (startPosition.equals("RED_RELIC")) {
            moveAlongWall(false, true, 150, 50);
        } else if (startPosition.equals("RED_MIDDLE")) {
            moveAlongWall(false, false, 60, 50);
        } else if (startPosition.equals("BLUE_RELIC")) {
            moveAlongWall(true, false, 150, 50);
        } else {
            moveAlongWall(true, true, 60, 50);
        }
    }*/

    /**
     * Knock the correct jewel down.
     */
    public void knockJewel() {
        telemetry.addData("knockJewel Method called", "");
        telemetry.update();
        steering.setSpeedRatio(MOVE_SPEED_RATIO);
        gunnerFunction.lowerJewelPusher();
        sleep(1000);
        double red = colorSensor.red();
        double blue = colorSensor.blue();

        boolean isRedTeam = startPosition.equals("RED_RELIC") || startPosition.equals("RED_MIDDLE");

        if(red > blue) {
            if(isRedTeam){
                pushJewel(JewelPosition.LEFT);
            }
            else{
                pushJewel(JewelPosition.RIGHT);
            }
        }
        else if (red < blue){
            if(isRedTeam){
                pushJewel(JewelPosition.RIGHT);
            }
            else{
                pushJewel(JewelPosition.LEFT);
            }
        } else {
            gunnerFunction.raiseJewelPusher();
            sleep(1000);
        }
    }

    /**
     * Push either the left or right jewel down.
     */
    public void pushJewel(JewelPosition jewelPosition) {
        telemetry.addData("Push side", jewelPosition);
        telemetry.update();
        if (jewelPosition == JewelPosition.LEFT) {
            telemetry.addData("reached checkpoint 1", "");
            telemetry.update();
            steering.stopAllMotors();
            steering.turn(-0.1);
            steering.finishSteering();
            sleep(200);
            steering.stopAllMotors();
            gunnerFunction.raiseJewelPusher();
            sleep(1000);
            steering.turn(0.1);
            steering.finishSteering();
            sleep(200);
            steering.stopAllMotors();
        } else {
            telemetry.addData("reached checkpoint 1", "");
            telemetry.update();
            steering.stopAllMotors();
            steering.turn(0.1);
            steering.finishSteering();
            sleep(200);
            steering.stopAllMotors();
            gunnerFunction.raiseJewelPusher();
            sleep(1000);
            steering.turn(-0.1);
            steering.finishSteering();
            sleep(200);
            steering.stopAllMotors();
        }
    }

    public void checkActive() {
        if (!opModeIsActive()) {
            stop();
        }
    }

    public enum JewelPosition {
        LEFT, RIGHT
    }
}