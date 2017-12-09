package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class UltrasonicFunction {
    private SmoothUltrasonic ultrasonicLeft;
    private SmoothUltrasonic ultrasonicRight;
    private SmoothUltrasonic ultrasonicRF;
    private SmoothUltrasonic ultrasonicLF;

    private Telemetry telemetry;

    private double SMOOTHNESS = 0;

    public UltrasonicFunction (UltrasonicSensor ultrasonicLeft, UltrasonicSensor ultrasonicRight, UltrasonicSensor ultrasonicRF, UltrasonicSensor ultrasonicLF, Telemetry telemetry) {
        this.ultrasonicLeft = new SmoothUltrasonic(ultrasonicLeft, SMOOTHNESS, telemetry);
        this.ultrasonicRight = new SmoothUltrasonic(ultrasonicRight, SMOOTHNESS, telemetry);
        this.ultrasonicRF = new SmoothUltrasonic(ultrasonicRF, SMOOTHNESS, telemetry);
        this.ultrasonicLF = new SmoothUltrasonic(ultrasonicLF, SMOOTHNESS, telemetry);
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

        public SmoothUltrasonic (UltrasonicSensor ultrasonicSensor, double smoothness, Telemetry telemetry) {
            this.ultrasonicSensor = ultrasonicSensor;
            this.smoothness = smoothness;
            double sum = 0;
            this.telemetry = telemetry;

            int successes = 0;
            double outputValue;

            //Average 20 readings from ultrasonic sensor: if they all ouput zero, use known velocity/acceleration values
            //DO MORE TESTING TO SEE HOW MANY READINGS WE REALLY NEED
            for (int attempt = 0; attempt < 20; attempt++) {
                outputValue = ultrasonicSensor.getUltrasonicLevel();
                if (outputValue != 0) {
                    sum += outputValue;
                    successes++;
                }
            }
            if (sum == 0) {
                telemetry.addData("Error: ", "Ultrasonic Sensor outputs zero all the time");
            } else {
                distance = sum/successes;
            }
            speed = 0;
            previousTickTime = System.currentTimeMillis();
        }

        public double getDistance () {
            double sum = 0;
            int successes = 0;
            double outputValue;

            //Average 5 readings from ultrasonic sensor: if they all output zero, use known velocity/acceleration values
            for (int attempt = 0; attempt < 5; attempt++) {
                outputValue = ultrasonicSensor.getUltrasonicLevel();
                if (outputValue == 0 || outputValue == 255 || Math.abs(outputValue-distance) > 50) {}
                else {
                    sum += outputValue;
                    successes++;
                }
            }
            long curTime = System.currentTimeMillis();
            long time = curTime-previousTickTime;
            if (sum == 0) {
                distance += speed * time;
                return distance;
            } else {
                double measuredDistance = sum/successes;
                double oldDistance = distance;
                double calculatedDistance = distance + speed * time;
                distance = (1 - smoothness) * measuredDistance + smoothness * calculatedDistance;
                speed = (distance-oldDistance)/(time);
                previousTickTime = curTime;

                telemetry.addData("speed: ", speed);
                telemetry.addData("distance: ", distance);
                telemetry.addData("time", time);
                telemetry.addData("calculated distance", calculatedDistance);
                return distance;
            }
        }
    }
}
