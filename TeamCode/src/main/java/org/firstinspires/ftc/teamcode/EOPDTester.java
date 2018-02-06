package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

@Autonomous
public class EOPDTester extends OpMode {
    //Using OpticalDistanceSensor does not work
    protected LightSensor leftEOPD;
    protected LightSensor rightEOPD;
    @Override
    public void init() {
        leftEOPD = this.hardwareMap.lightSensor.get("leftEOPD");
        rightEOPD = this.hardwareMap.lightSensor.get("rightEOPD");
    }

    @Override
    public void loop() {
        telemetry.addData("leftrgld", leftEOPD.getRawLightDetected());
        telemetry.addData("rightrgld", rightEOPD.getRawLightDetected());
        telemetry.update();
    }
}
