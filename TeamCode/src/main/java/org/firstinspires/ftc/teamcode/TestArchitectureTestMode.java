package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Test Architecture Test")
public class TestArchitectureTestMode extends LinearOpMode {
    enum SubMode {
        MAIN ("Main"),
        TEST_A ("Test A"),
        TEST_B ("Test B"),
        TEST_C ("Test C");

        private final String name;
        private static final SubMode[] values = values();

        SubMode(String name) {
            this.name = name;
        }

        public String getName() {
            return name;
        }

        public static SubMode fromOrdinal(int ordinal) {
            return values[ordinal % values.length];
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.setAutoClear(false);
        SubMode subMode = selectMode();
        waitForStart(); // just for safety; not strictly needed
        switch (subMode) {
            case MAIN:
                runMainSubOpMode();
                break;
            case TEST_A:
                runTestA();
                break;
            case TEST_B:
                runTestB();
                break;
            case TEST_C:
                runTestC();
                break;
        }
    }

    private void runTestC() {
        telemetry.addData("Running", "TEST_C");
        telemetry.update();
    }

    private void runTestB() {
        telemetry.addData("Running", "TEST_B");
        telemetry.update();
    }

    private void runTestA() {
        telemetry.addData("Running", "TEST_A");
        telemetry.update();
    }

    private void runMainSubOpMode() {
        telemetry.addData("Running", "MAIN");
        telemetry.update();
    }

    private SubMode selectMode() {
        int modeIndex = 0;
        while (!isStarted() && opModeIsActive()) {
            if (gamepad1.dpad_left) modeIndex--;
            if (gamepad1.dpad_right) modeIndex++;
            telemetry.addData("Selected mode", SubMode.fromOrdinal(modeIndex));
            telemetry.update();
        }
        return SubMode.fromOrdinal(modeIndex);
    }
}
