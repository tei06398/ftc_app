package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import org.firstinspires.ftc.robotcore.external.Telemetry;

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
        this.telemetry = telemetry;
    }
    
    public UltrasonicFunction (UltrasonicSensor ultrasonicLeft, UltrasonicSensor ultrasonicRight, UltrasonicSensor ultrasonicRF, UltrasonicSensor ultrasonicLF, Telemetry telemetry) {
        this.ultrasonicLeft = new SmoothUltrasonic(ultrasonicLeft, telemetry);
        this.ultrasonicRight = new SmoothUltrasonic(ultrasonicRight, telemetry);
        this.ultrasonicRF = new SmoothUltrasonic(ultrasonicRF, telemetry);
        this.ultrasonicLF = new SmoothUltrasonic(ultrasonicLF, telemetry);
        this.telemetry = telemetry;
    }

    public double getLeft() {return ultrasonicLeft.getDistance();}
    public double getRight() {return ultrasonicRight.getDistance();}
    public double getLF() {return ultrasonicLF.getDistance();}
    public double getRF() {return ultrasonicRF.getDistance();}


    public static class SmoothUltrasonic {
        private UltrasonicSensor ultrasonicSensor;
        private double distance;

        public SmoothUltrasonic (UltrasonicSensor ultrasonicSensor, Telemetry telemetry) {
            this.ultrasonicSensor = ultrasonicSensor;
            getDistance();
        }

        public double getDistance () {
            double outputValue;

            for (int attempt = 0; attempt < 20; attempt++) {
                outputValue = ultrasonicSensor.getUltrasonicLevel();
                if (outputValue != 0 && outputValue != 255 && outputValue != 127) {
                    distance = outputValue;
                    return outputValue;
                }
            }
            return distance;
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
