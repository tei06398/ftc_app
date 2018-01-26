package org.firstinspires.ftc.teamcode;

/**
 * An interface for classes that can build Steering objects.
 * @param <T> The type of Steering object that this class can build.
 */
public interface SteeringBuilder<T extends Steering<T>> {
    T build();
    SteeringBuilder<T>  noMotion();
}
