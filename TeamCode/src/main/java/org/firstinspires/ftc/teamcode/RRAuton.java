package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.hardware.camera2.CameraAccessException;
import android.widget.TextView;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import com.vuforia.Frame;
import com.vuforia.Image;
import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.*;
import org.opencv.android.OpenCVLoader;
import org.opencv.core.*;
import org.opencv.videoio.VideoCapture;

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
    protected UltrasonicSensor ultrasonic1 = null;
    protected VideoCapture camera = null;


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
        this.ultrasonic1 = this.hardwareMap.ultrasonicSensor.get("ultrasonic1");

        this.motorLF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.motorRF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.motorLB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.motorRB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //Makes RobotDriving Object
        RobotDriving robotDriving = new RobotDriving(motorLF, motorLB, motorRF, motorRB, telemetry);

        //Gets TimedSteering Object
        RobotDriving.TimedSteering ts = robotDriving.getTimedSteering();

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


        telemetry.addData("Reached checkpoint", "1");
        telemetry.update();
        //((TextView)((FtcRobotControllerActivity) this.hardwareMap.appContext).findViewById(R.id.tvStatusString)).setText("TEST TEST");
        //OpenCVLoader.initDebug(); // if this fails, try commented line below

        //System.loadLibrary(Core.NATIVE_LIBRARY_NAME); // if crashes at this line, change parameter to libopencv_java3 or opencv_java3

        try {
            boolean load = OpenCVLoader.initDebug();
            telemetry.addData("Reached checkpoint", load);
            telemetry.update();
            if(load){
                System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
                System.loadLibrary("opencv_java3");
            }
            telemetry.addData("Reached checkpoint", "2.5");
            telemetry.update();
            camera = new VideoCapture();
        }
        catch (Exception e){
            telemetry.addData("Error: ", e.getMessage());
            telemetry.update();
        }
        telemetry.addData("Reached checkpoint", "3");
        telemetry.update();
        Mat mat = new Mat();
        telemetry.addData("Reached checkpoint", "4");
        telemetry.update();

        if(!camera.isOpened()) {
            telemetry.addData("Camera working", "false");
        } else {
            telemetry.addData("Camera working", "true");
            while (!gamepad2.x) {
                try {
                    Thread.sleep(1000);
                } catch (InterruptedException e) {
                    // Shouldn't happen
                }

                if (camera.read(mat)) {
                    final int HEIGHT = mat.rows();
                    final int WIDTH = mat.cols();

                    int ballCol = (int) (WIDTH * 0.67);
                    int ballRow = (int) (HEIGHT * 0.75);

                    double blueWeight = 0;
                    double redWeight = 0;
                    for (int row = (int) (HEIGHT * 0.5); row < HEIGHT; row += 10) {
                        for (int col = (int) (WIDTH * 0.33); col < WIDTH; col += 10) {
                            double distWeight = Math.abs(ballRow - row) + Math.abs(ballCol - col);

                            blueWeight += mat.get(row, col)[0] / (distWeight + 20);
                            redWeight += mat.get(row, col)[2] / (distWeight + 20);
                        }
                    }
                    telemetry.addData("Red: ", redWeight);
                    telemetry.addData("Blue: ", blueWeight);
                    telemetry.addData("Color: ", redWeight > blueWeight ? "RED" : "BLUE");
                    telemetry.update();
                }
            }
            camera.release();
        }

        //Activate the VuMark Dataset as Current Tracked Object
        relicTrackables.activate();

        //Get a semi-reliable reading of the Pictograph
        while(this.opModeIsActive()) {
            int total = 0;
            char pictograph = 'E';

            while (total<3) {
                pictograph = readVuMark(relicTemplate);
                if (pictograph != '!') {
                    total = 3;
                } else {
                    total++;
                }
            }
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
            telemetry.update();
        }
    }
}
