package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.robotcore.external.Consumer;

public class SleepFunction {
    Consumer<Long> sleepCallback;

    public SleepFunction(Consumer<Long> sleepCallback) {
        this.sleepCallback = sleepCallback;
    }

    public void sleep(long milliseconds) {
        sleepCallback.accept(milliseconds);
    }
}
