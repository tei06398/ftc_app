package org.firstinspires.ftc.teamcode;

import android.util.Log;

public class NamedLog {
    private String tag;

    public NamedLog(String tag) {
        this.tag = tag;
    }

    public NamedLog child(String subtag) {
        return new NamedLog(tag + "/" + subtag);
    }

    public void v(String msg) {
        Log.v(tag, msg);
    }

    public void d(String msg) {
        Log.d(tag, msg);
    }

    public void i(String msg) {
        Log.i(tag, msg);
    }

    public void w(String msg) {
        Log.w(tag, msg);
    }

    public void e(String msg) {
        Log.e(tag, msg);
    }
}
