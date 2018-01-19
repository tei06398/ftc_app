package org.firstinspires.ftc.teamcode.util.drive;

public interface SteeringBuilder<T extends Steering<T>> {
    T build();
    SteeringBuilder<T>  noMotion();
}
