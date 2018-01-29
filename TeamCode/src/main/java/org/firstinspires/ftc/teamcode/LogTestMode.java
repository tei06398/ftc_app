package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Log Test")
public class LogTestMode extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        RobotLog log = new RobotLog("NiskyRobot", telemetry);
        RobotLog log2 = log.child("Foo");
        RobotLog log3 = log2.child("Bar");
        NumberLog numberLog = new NumberLog(log3, 20);
        int i = 0;
        while (opModeIsActive()) {
            sleep(100);
            numberLog.addItem(i++);
        }
        numberLog.flush();
    }
}
