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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.*;

import java.io.ByteArrayInputStream;
import java.io.File;

/**
 * Created 11/13/2017
 */
@Autonomous(name = "Relic Recovery Official Auton Mode")
public class RRAuton extends LinearOpMode {
    //Declares Motors
    protected DcMotor motorLF = null;
    protected DcMotor motorRF = null;
    protected DcMotor motorLB = null;
    protected DcMotor motorRB = null;
    protected UltrasonicSensor ultrasonicLeft = null;
    protected UltrasonicSensor ultrasonicRight = null;
    protected UltrasonicSensor ultrasonicRF = null;
    protected UltrasonicSensor ultrasonicLF = null;

    protected RobotDriving robotDriving;
    protected RobotDriving.Steering steering;

    protected UltrasonicFunction ultrasonicFunction;

    SharedPreferences sharedPref;
    protected String startPosition; //RED_RELIC, RED_MIDDLE, BLUE_RELIC, BLUE_MIDDLE

    //Required distance from wall at the beginning
    protected int wallDistance = 30;

    //protected VideoCapture camera = null;

    private VuforiaLocalizer vuforia;
    public static String jewelResult = "";
    public static final void setJewelResult(String result) {
        jewelResult = result;
    }
    public static String getJewelResult() {
        return jewelResult;
    }

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

        this.motorLF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.motorRF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.motorLB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.motorRB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        sharedPref = PreferenceManager.getDefaultSharedPreferences(this.hardwareMap.appContext);
        startPosition = sharedPref.getString("auton_start_position","RED_RELIC");

        // RobotDriving instantiation
        robotDriving = new RobotDriving(motorLF, motorLB, motorRF, motorRB, telemetry);
        steering = robotDriving.getSteering();

        //Ultrasonic function instantiation
        ultrasonicFunction = new UltrasonicFunction(ultrasonicLeft, ultrasonicRight, ultrasonicRF, ultrasonicLF, telemetry);

        //Tell Vuforia to display video feed
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        //Disable Video Feed with the following:
        //VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();


        //Set License Key, tell Vuforia to use back Camera, finalize Vuforia Parameters for Localizer
        parameters.vuforiaLicenseKey = "AUrG/9b/////AAAAGYb5udEIt0W7p8AYwNbs5lUO4Ojghb2IvJN64Q6ZnRvUfbl59Au5c48n2lykKrsfZgYx9m2HpOgdNFmLaxhilDQIc0mmohbk5IjXvKkGGJR4OiqNtqYVDncXZpb/esaPeFTLtkbJFAlEs+oPwcAKoO5FctEuFyEgz1IJc6/MRphweDiXuJ86Rqs81UVeOlNXdr3QtZazJcViHHeWkv5pJUvefJWbjXSZvOZFlITDaTbyPIibQsHsbDg+B0IBMHK+d8fs58anmJ1VJEoQ9Xo0YDpHgosgx996zfSjhAK8YcIxLQdCkFbJtQtkpAl14jz+Yz86QvP8AMtFeMRuIYrPWhQvPJ3VU/Jm1Qmz+p/jbJ8k";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        //Load in the VuMark dataset from assets
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);

        //Wait for OpMode Init Button to be Pressed
        waitForStart();

        //Activate the VuMark Dataset as Current Tracked Object
        relicTrackables.activate();

        //Get a semi-reliable reading of the Pictograph
        int total = 0;
        char pictograph = 'E';

        while (total < 3) {
            pictograph = readVuMark(relicTemplate);
            if (pictograph != '!') {
                total = 3;
            } else {
                total++;
            }
        }
        if (pictograph == '!') {
            telemetry.addData("Pictograph: ", "NaN - Björk 403: Pictograph Not Accessible");
            //Displays in the event that 3/3 times, the data returned by readVuMark() has been 1L,1C,1R, not allowing for a logical interpretation.
        } else if (pictograph == 'l') {
            telemetry.addData("Pictograph: ", "Left");
        } else if (pictograph == 'r') {
            telemetry.addData("Pictograph: ", "Right");
        } else if (pictograph == 'c') {
            telemetry.addData("Pictograph: ", "Center");
        } else {
            telemetry.addData("Pictograph: ", "NaN - Björk 500: Internal Pictograph Error");
            //Displays only if the initial value of pictograph remains unchanged, which shouldn't occur.
        }
        telemetry.update();

        //Detect whiffle ball location
        Camera camera = Camera.open();
        camera.takePicture(null, null,
                new Camera.PictureCallback() {
                    @Override
                    public void onPictureTaken(byte[] data, Camera camera) {
                        ByteArrayInputStream bais = new ByteArrayInputStream(data);
                        Bitmap bmp = BitmapFactory.decodeStream(bais);
                        int width = bmp.getWidth();
                        int height = bmp.getHeight();

                        double redWeight = 0;
                        double blueWeight = 0;

                        double red;
                        double blue;
                        double green;
                        double weight;

                        for (int row = 2 * height / 3; row < height; row += 10) {
                            for (int col = width/2; col < width; col += 10) {
                                red = Color.red(bmp.getPixel(row, col));
                                blue = Color.blue(bmp.getPixel(row, col));
                                green =  Color.green(bmp.getPixel(row, col));
                                if (red > blue && red > green) {
                                    weight = Math.pow(row - 3 * height / 4, 2) + Math.pow(col - 2 * width / 3, 2);
                                    redWeight += red * weight;
                                }
                                else if (blue > red && blue > green) {
                                    weight = Math.pow(row - 3 * height / 4, 2) + Math.pow(col - 2 * width / 3, 2);
                                    blueWeight += blue * weight;
                                }
                            }
                        }
                        if (redWeight > blueWeight) {
                            setJewelResult("red");
                        } else {
                            setJewelResult("blue");
                        }
                    }
                });

        //TODO: Replace the Telemetry with actual boops when we have the booper -Seth
        if(getJewelResult().equals("red")) {
            //RED
            if (startPosition.equals("RED_RELIC") || startPosition.equals("RED_MIDDLE")) {
                //thing.knockright();
                telemetry.addData("Knock: ", "RIGHT");
            } else {
                telemetry.addData("Knock: ", "LEFT");
            }
        } else if (getJewelResult().equals("blue")) {
            if (startPosition.equals("BLUE_RELIC") || startPosition.equals("BLUE_MIDDLE")) {
                telemetry.addData("Knock: ", "RIGHT");
            } else {
                telemetry.addData("Knock: ", "LEFT");
            }
        } else {
            telemetry.addData("Knock: ", "NaN - Björk 404: Ball Not Found");
            telemetry.update();
        }

        //Extend whiffle ball thing

        //Starting case Red Recovery
        //if (STARTING CASE ENUM) {
        double forwardWeight = 0;
        double clockwiseTurnWeight = 0;
        while (ultrasonicFunction.getRight() < 100) {
            steering.moveDegrees(180, 1);

            //Determine robot turning off course
            if (ultrasonicFunction.getLF() + 1 < ultrasonicFunction.getRF()) {
                clockwiseTurnWeight -= 0.01;
            } else if (ultrasonicFunction.getRF() + 1 < ultrasonicFunction.getLF()){
                clockwiseTurnWeight += 0.01;
            }

            //Determine robot drifting off course
            if ((ultrasonicFunction.getLF() + ultrasonicFunction.getRF()) / 2 + 1 < wallDistance) {
                forwardWeight += 0.01;
            } else if ((ultrasonicFunction.getLF() + ultrasonicFunction.getRF()) / 2 - 1 > wallDistance) {
                forwardWeight -= 0.01;
            }

            steering.moveDegrees(180, 1);
            if (forwardWeight > 0) {steering.moveDegrees(90, forwardWeight);}
            else {steering.moveDegrees(270, -forwardWeight);}
            steering.turn(clockwiseTurnWeight);
            steering.finishSteering();
        }
    }
}