package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;

@Autonomous(name = "Autonomous Test")
public class AutonFunctionTester extends OpMode {
    protected UltrasonicFunction ultrasonicFunction;
    protected ColorSensor colorSensor;
    protected GunnerFunction gunnerFunction;

    @Override
    public void init() {
        ultrasonicFunction = new UltrasonicFunction(hardwareMap, RobotLog.getRootInstance(telemetry));
        colorSensor = hardwareMap.colorSensor.get("jewelTipper");
        gunnerFunction = new GunnerFunction(hardwareMap, telemetry);
        gunnerFunction.disablePwm(hardwareMap);
        gunnerFunction.reset();
    }

    @Override
    public void loop() {
        telemetry.addData("Red", colorSensor.red());
        telemetry.addData("Blue", colorSensor.blue());
        telemetry.addData("Alpha", colorSensor.alpha());
        ultrasonicFunction.printTestData();
        telemetry.update();

    }
}
