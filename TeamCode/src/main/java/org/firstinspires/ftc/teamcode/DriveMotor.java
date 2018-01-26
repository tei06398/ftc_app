package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.WeightedValue;

/**
 * This class wraps a regular motor and adds utilities to it.
 * The main feature is that their powers are smoothed so when you "apply" power to them,
 * the power is gradually shifted.
 */
public class DriveMotor {
    private DcMotor motor;
    private WeightedValue acceleration;

    public DriveMotor(DcMotor motor, double smoothness) {
        this.motor = motor;
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        acceleration = new WeightedValue(smoothness);
    }

    /**
     * Apply a power to the motor.
     * @param power Power, between -1 and 1 as with normal motors.
     */
    public void applyPower(double power) {
        this.motor.setPower(acceleration.applyValue(power));
    }
}
