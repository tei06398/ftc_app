package org.firstinspires.ftc.teamcode;

public class WeightedValue {
    private double value = 0;
    private double smoothness;

    public WeightedValue(double smoothness) {
        this.smoothness = smoothness;
    }

    public double applyValue(double newValue) {
        if (value < newValue) {
            value = value + Math.min(newValue-value, smoothness);
        }
        else {
            value = value - Math.min(value-newValue, smoothness);
        }
        return value;
    }
}
