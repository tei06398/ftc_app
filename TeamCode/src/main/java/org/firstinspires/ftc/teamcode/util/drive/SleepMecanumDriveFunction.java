package org.firstinspires.ftc.teamcode.util.drive;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.SleepFunction;

public class SleepMecanumDriveFunction extends MecanumDriveFunction {
    private SleepFunction sleepFunction;

    public SleepMecanumDriveFunction(HardwareMap hardwareMap, Telemetry telemetry, SleepFunction sleepFunction) {
        super(hardwareMap, telemetry);
        this.sleepFunction = sleepFunction;
    }

    public SleepMecanumDriveFunction(HardwareMap hardwareMap, Telemetry telemetry, SleepFunction sleepFunction, double smoothness) {
        super(hardwareMap, telemetry, smoothness);
        this.sleepFunction = sleepFunction;
    }

    public void steerFor(SteeringBuilder<DiagonalVectorSteering> steeringBuilder, long milliseconds) {
        steer(steeringBuilder);
        sleepFunction.sleep(milliseconds);
        stopMotors();
    }
}
