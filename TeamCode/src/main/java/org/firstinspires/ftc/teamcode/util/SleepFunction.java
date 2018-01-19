package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.robotcore.external.Consumer;

/**
 * This function class can be passed into other function classes, allowing them to sleep.
 */
public class SleepFunction {
    Consumer<Long> sleepCallback;

    public SleepFunction(Consumer<Long> sleepCallback) {
        this.sleepCallback = sleepCallback;
    }

    public void sleep(long milliseconds) {
        sleepCallback.accept(milliseconds);
    }
}
