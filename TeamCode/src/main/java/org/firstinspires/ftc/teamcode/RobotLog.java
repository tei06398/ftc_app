package org.firstinspires.ftc.teamcode;

import android.util.Log;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class RobotLog {
    private String tag;
    private Telemetry telemetry;

    public static RobotLog getRootInstance(Telemetry telemetry) {
        return new RobotLog("NiskyRobot", telemetry);
    }

    public RobotLog(String tag, Telemetry telemetry) {
        this.tag = tag;
        this.telemetry = telemetry;
    }

    public RobotLog child(String subtag) {
        return new RobotLog(tag + "/" + subtag, telemetry);
    }

    public NumberLog getNumberLog() {
        return new NumberLog(this);
    }

    public NumberLog getNumberLog(int bufferSize) {
        return new NumberLog(this, bufferSize);
    }

    public void v(String msg) {
        Log.v(tag, msg);
    }

    public void d(String msg) {
        Log.d(tag, msg);
        telemetry.log().add("D/" + msg);
    }

    public void i(String msg) {
        Log.i(tag, msg);
        telemetry.log().add("I/" + msg);
    }

    public void w(String msg) {
        Log.w(tag, msg);
        telemetry.log().add("W/" + msg);
    }

    public void e(String msg) {
        Log.e(tag, msg);
        telemetry.log().add("E/" + msg);
    }

    public Telemetry getTelemetry() {
        return telemetry;
    }
}
