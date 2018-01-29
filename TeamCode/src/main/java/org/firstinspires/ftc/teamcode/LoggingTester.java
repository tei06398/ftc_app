package org.firstinspires.ftc.teamcode;

import android.util.Log;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Log Test")
public class LoggingTester extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        int i = 0;
        while (opModeIsActive()) {
            sleep(100);
            Log.d("Robot", Integer.toString(i));
            i++;
        }
    }
}
