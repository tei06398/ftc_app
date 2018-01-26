package org.firstinspires.ftc.teamcode;

/**
 * An interface for objects that represent a steering, a set of desired motor powers that can be applied to a
 * DrivingFunction.
 * @param <T> The same type as implementors of this interface.
 */
public interface Steering<T> {
    T scale(double maxPower);
}
