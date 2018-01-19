package org.firstinspires.ftc.teamcode.util.drive;

import org.firstinspires.ftc.teamcode.util.SleepFunction;

public interface DriveFunction<T extends Steering<T>> {
    void steer(T steering);
    void steer(SteeringBuilder<T> steeringBuilder);
}
