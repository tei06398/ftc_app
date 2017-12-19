package org.firstinspires.ftc.teamcode;

import com.qualcomm.ftccommon.FtcRobotControllerService;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.FtcRobotControllerServiceState;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

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
