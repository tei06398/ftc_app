package org.firstinspires.ftc.teamcode;

public class NumericalAcceleration {
    private double amount = 0;
    private double weightToPrevious;

    public NumericalAcceleration(double weightToPrevious) {
        this.weightToPrevious = weightToPrevious;
    }

    public double applyAmount(double newAmount) {
        amount = amount * weightToPrevious + newAmount * (1 - weightToPrevious);
        return amount;
    }
}
