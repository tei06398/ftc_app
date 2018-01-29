package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@TeleOp(name = "Ultrasonic Test")
public class UltrasonicTestMode extends OpMode {
    private UltrasonicFunction ultrasonicFunction;
    
    @Override
    public void init() {
        ultrasonicFunction = new UltrasonicFunction(hardwareMap, RobotLog.getRootInstance(telemetry));
    }
    
    @Override
    public void loop() {
        ultrasonicFunction.test();
    }
}
