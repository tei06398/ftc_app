package org.firstinspires.ftc.teamcode;

/**
 * An interface for drive functions that can be steered with a Steering object.
 * @param <T> The type of the Steering object.
 */
public interface Steerable<T extends Steering<T>> {
    void steer(T steering);
    void steer(SteeringBuilder<T> steeringBuilder);
}
