package org.firstinspires.ftc.teamcode;

public class WeightedValue {
    private double value = 0;
    private double smoothness;

    public WeightedValue(double smoothness) {
        this.smoothness = smoothness;
    }

    public double applyValue(double newValue) {
        value = value * smoothness + newValue * (1 - smoothness);
        return value;
    }
}
