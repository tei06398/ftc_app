package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

@TeleOp(name = "Color Test Mode")
public class ColorSensorTestMode extends OpMode {

    protected ColorSensor colorSensor;

    public void loop(){

        telemetry.addData("Red", colorSensor.red());
        telemetry.addData("Blue", colorSensor.blue());
        telemetry.addData("Alpha", colorSensor.alpha());
        telemetry.update();
    }
    public void init(){
        colorSensor =  this.hardwareMap.colorSensor.get("colorSensor");
    }

}
