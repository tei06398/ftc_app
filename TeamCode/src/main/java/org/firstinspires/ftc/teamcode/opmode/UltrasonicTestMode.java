package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.util.UltrasonicFunction;

@TeleOp(name = "Ultrasonic Test")
public class UltrasonicTestMode extends OpMode {
    private UltrasonicFunction ultrasonicFunction;
    
    @Override
    public void init() {
        ultrasonicFunction = new UltrasonicFunction(hardwareMap, telemetry);
    }
    
    @Override
    public void loop() {
        ultrasonicFunction.test();
    }
}
