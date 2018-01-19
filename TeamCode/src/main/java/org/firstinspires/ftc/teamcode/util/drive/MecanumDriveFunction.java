package org.firstinspires.ftc.teamcode.util.drive;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Encapsulates robot driving utilities.
 */
public class MecanumDriveFunction implements Steerable<DiagonalVectorSteering> {
    private DriveMotor lf; // stands for left front
    private DriveMotor lb; // stands for left back
    private DriveMotor rf; // stands for right front
    private DriveMotor rb; // stands for right back

    private Telemetry telemetry;

    private double speedRatio = NORMAL_SPEED_RATIO;

    public static final double MAX_SPEED_RATIO = 1;
    public static final double NORMAL_SPEED_RATIO = 0.5;
    public static final double MIN_SPEED_RATIO = 0.3;

    public static final double DEFAULT_SMOOTHNESS = 0.1;

    public MecanumDriveFunction(HardwareMap hardwareMap, Telemetry telemetry) {
        this(hardwareMap, telemetry, DEFAULT_SMOOTHNESS);
    }

    public MecanumDriveFunction(HardwareMap hardwareMap, Telemetry telemetry, double smoothness) {
        this.lf = new DriveMotor(hardwareMap.dcMotor.get("lfMotor"), smoothness);
        this.lb = new DriveMotor(hardwareMap.dcMotor.get("lbMotor"), smoothness);
        this.rf = new DriveMotor(hardwareMap.dcMotor.get("rfMotor"), smoothness);
        this.rb = new DriveMotor(hardwareMap.dcMotor.get("rbMotor"), smoothness);
        this.telemetry = telemetry;
    }

    public double getSpeedRatio() {
        return speedRatio;
    }

    public void setSpeedRatio(double speedRatio) {
        this.speedRatio = speedRatio;
    }

    @Override
    public void steer(DiagonalVectorSteering steering) {
        steering = steering.scale(speedRatio);

        lf.applyPower(steering.getLf());
        lb.applyPower(steering.getLb());
        rf.applyPower(steering.getRf());
        rb.applyPower(steering.getRb());
    }

    @Override
    public void steer(SteeringBuilder<DiagonalVectorSteering> steeringBuilder) {
        steer(steeringBuilder.build());
    }

    public void stopMotors() {
        steer(new DiagonalVectorSteeringBuilder().noMotion());
    }
}
