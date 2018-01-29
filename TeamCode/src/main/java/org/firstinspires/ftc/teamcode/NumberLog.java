package org.firstinspires.ftc.teamcode;

import java.util.ArrayList;

public class NumberLog {
    private ArrayList<Double> items;
    private ArrayList<Double> allItems;

    private RobotLog robotLog;
    private int bufferSize;
    private static int DEFAULT_BUFFER_SIZE = 10;

    public NumberLog(RobotLog robotLog) {
        this(robotLog, DEFAULT_BUFFER_SIZE);
    }

    public NumberLog(RobotLog robotLog, int bufferSize) {
        this.robotLog = robotLog;
        this.bufferSize = bufferSize;
        items = new ArrayList<>();
        allItems = new ArrayList<>();
    }

    public void addItem(double item) {
        items.add(item);
        allItems.add(item);
        if (items.size() == bufferSize) flush();
    }

    public void flush() {
        StringBuilder msg = new StringBuilder();

        StringBuilder raw = new StringBuilder();
        for (double item : items) {
            raw.append(item).append(' ');
        }

        double mean = calculateMean();
        appendData(msg, "mean", String.valueOf(mean));
        appendData(msg, "standard deviation", String.valueOf(calculateStandardDeviation(mean)));
        appendData(msg, "raw", raw);

        robotLog.d(msg.toString());

        items.clear();
    }

    private void appendData(StringBuilder sb, CharSequence key, CharSequence value) {
        sb.append(key).append('=').append(value).append(';');
    }

    private double calculateMean() {
        double sum = 0;
        for (double item : allItems) {
            sum += item;
        }
        return sum / allItems.size();
    }

    private double calculateStandardDeviation(double mean) {
        double sumOfDifferences = 0;
        for (double item : allItems) {
            sumOfDifferences += Math.pow(item - mean, 2);
        }
        return Math.sqrt(sumOfDifferences / allItems.size());
    }
}
