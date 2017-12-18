package org.firstinspires.ftc.teamcode;

import android.content.SharedPreferences;
import android.preference.PreferenceManager;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Sleep Test")
public class SleepTestAuton extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        telemetry.setAutoClear(false)
        telemetry.addData("Sleep", "begins");
        sleep(2000);
        telemetry.addData("Sleep", "ends");
        sleep(10000);
    }
}
