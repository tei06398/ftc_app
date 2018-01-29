package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Number Log Test")
public class NumberLogTestMode extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        NumberLog numberLog = new NumberLog("NumberLog test", 20);
        int i = 0;
        while (opModeIsActive()) {
            sleep(100);
            numberLog.addItem(i++);
        }
        numberLog.flush();
    }
}
