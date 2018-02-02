package org.firstinspires.ftc.teamcode;

public class RobotUtil {
    public static double clipRange(double min, double max, double value) {
        // Fix swapped ordering
        if (min > max) {
            double oldMax = max;
            max = min;
            min = oldMax;
        }
        return Math.min(max, Math.max(value, min));
    }
}
