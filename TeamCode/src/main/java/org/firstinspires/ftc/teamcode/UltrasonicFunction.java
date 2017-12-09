package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class UltrasonicFunction {
    private SmoothUltrasonic ultrasonicLeft;
    private SmoothUltrasonic ultrasonicRight;
    private SmoothUltrasonic ultrasonicRF;
    private SmoothUltrasonic ultrasonicLF;

    private Telemetry telemetry;

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
        private double speed;
        private double smoothness;
        private long previousTickTime;
        private Telemetry telemetry;
        private boolean hasBegun;

        public SmoothUltrasonic (UltrasonicSensor ultrasonicSensor, Telemetry telemetry) {
            this.ultrasonicSensor = ultrasonicSensor;
            double sum = 0;
            this.telemetry = telemetry;

            int successes = 0;
            double outputValue;

            //Average 20 readings from ultrasonic sensor: if they all ouput zero, use known velocity/acceleration values
            //DO MORE TESTING TO SEE HOW MANY READINGS WE REALLY NEED
            for (int attempt = 0; attempt < 20; attempt++) {
                outputValue = ultrasonicSensor.getUltrasonicLevel();
                if (outputValue == 0 || outputValue == 255) {}
                else {
                    sum += outputValue;
                    successes++;
                }
            }
            if (sum == 0) {
                distance = 128;
                hasBegun = false;
                telemetry.addData("Error: ", "Ultrasonic Sensor outputs zero all the time");
            } else {
                distance = sum / successes;
                hasBegun = true;
            }
            double x = 0;
            for (int i = 0; i < 1000000; i++) {
                x = Math.sqrt(i);
            }
            double a = x;
            previousTickTime = System.currentTimeMillis();
        }

        public double getDistance () {
            double sum = 0;
            int successes = 0;
            double outputValue;

            //Average 5 readings from ultrasonic sensor: if they all output zero, use known velocity/acceleration values
            for (int attempt = 0; attempt < 10; attempt++) {
                outputValue = ultrasonicSensor.getUltrasonicLevel();
                //telemetry.addData("output value: ", outputValue);
                if (outputValue == 0 || outputValue == 255 || (hasBegun && Math.abs(outputValue-distance) > 50)) {}
                else {
                    sum += outputValue;
                    successes++;
                }
                double x = 0;
                for (int i = 0; i < 1000000; i++) {
                    x = Math.sqrt(i);
                }
                double a = x;
            }
            //telemetry.addData("sum: ", sum);
            //telemetry.addData("successes: ", successes);

            if (sum == 0) {
                return distance;
            } else {
                distance = sum / successes;
                return distance;
            }
        }
    }
}
