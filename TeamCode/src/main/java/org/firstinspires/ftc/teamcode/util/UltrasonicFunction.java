package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.LegacyModule;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * This class contains utilities that read from ultrasonic sensors.
 */
public class UltrasonicFunction {
    private SmoothUltrasonic ultrasonicLeft;
    private SmoothUltrasonic ultrasonicRight;
    private SmoothUltrasonic ultrasonicRF;
    private SmoothUltrasonic ultrasonicLF;

    private Telemetry telemetry;
    
    public UltrasonicFunction (HardwareMap hardwareMap, Telemetry telemetry) {
        this.ultrasonicLeft = new SmoothUltrasonic(hardwareMap.ultrasonicSensor.get("ultrasonicLeft"), telemetry); //module 2, port 1
        this.ultrasonicRight = new SmoothUltrasonic(hardwareMap.ultrasonicSensor.get("ultrasonicRight"), telemetry);//module 2, port 2
        this.ultrasonicLF = new SmoothUltrasonic(hardwareMap.ultrasonicSensor.get("ultrasonicLF"), telemetry); //module 3, port 3
        this.ultrasonicRF = new SmoothUltrasonic(hardwareMap.ultrasonicSensor.get("ultrasonicRF"), telemetry); //module 4, port 4

        hardwareMap.legacyModule.get("Legacy Module 2").enable9v(4, true);
        hardwareMap.legacyModule.get("Legacy Module 2").enable9v(5, true);


        this.telemetry = telemetry;
    }

    public double getLeft() {
        return ultrasonicLeft.getDistance();
    }

    public double getRight() {
        return ultrasonicRight.getDistance();
    }
    public double getLF() {
        return ultrasonicLF.getDistance();
    }

    public double getRF() {
        return ultrasonicRF.getDistance();
    }

    public void setLeft(double input) {
        ultrasonicLeft.setDistance(input);
    }

    public void setLF(double input) {
        ultrasonicLF.setDistance(input);
    }

    public void setRF(double input) {
        ultrasonicRF.setDistance(input);
    }

    public void setRight(double input) {
        ultrasonicRight.setDistance(input);
    }

    /**
     * Adds utilities to an UltrasonicSensor.
     */
    public static class SmoothUltrasonic {
        private UltrasonicSensor ultrasonicSensor;
        private double distance;
        private Telemetry telemetry;

        public SmoothUltrasonic (UltrasonicSensor ultrasonicSensor, Telemetry telemetry) {
            this.ultrasonicSensor = ultrasonicSensor;
            this.telemetry = telemetry;

            distance = 255;
            getDistance();
        }

        public double getDistance() {
            double outputValue;
            double sum = 0;
            double successes = 0;
            for (int attempt = 0; attempt < 20; attempt++) {
                outputValue = ultrasonicSensor.getUltrasonicLevel();
                if (outputValue != 0 && outputValue != 255 && outputValue != 127) {
                    sum += outputValue;
                    successes++;
                }
            }
            telemetry.addData("Number of successes: ", successes);
            if (successes > 0) {
                distance = sum/successes;
            }
            return distance;
        }

        public void setDistance (double input) {
            distance = input;
        }
    }
    
    public void test() {
        printTestData();
        telemetry.update();
    }

    public void printTestData() {
        telemetry.addData("Ultrasonic LEFT", getLeft());
        telemetry.addData("Ultrasonic RIGHT", getRight());
        telemetry.addData("Ultrasonic LEFT_FRONT", getLF());
        telemetry.addData("Ultrasonic RIGHT_FRONT", getRF());
    }
}
