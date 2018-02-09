package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Test Architecture Test")
public class TestArchitectureTestMode extends LinearOpMode {
    enum SubMode {
        MAIN,
        TEST_A,
        TEST_B,
        TEST_C;

        private static final SubMode[] values = values();

        public static SubMode fromOrdinal(int ordinal) {
            return values[ordinal % values.length];
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.setAutoClear(false);
        int modeIndex = 0;
        SubMode subMode;
        while (opModeIsActive()) {
            if (gamepad1.dpad_left) modeIndex--;
            if (gamepad1.dpad_right) modeIndex++;
            telemetry.addData("Sub mode", SubMode.fromOrdinal(modeIndex).toString());
            telemetry.update();
            if (gamepad1.x) {
                subMode = SubMode.fromOrdinal(modeIndex);
                waitForStart();
                runSubMode(subMode);
            }
            if (gamepad1.y) {
                requestOpModeStop();
            }
        }
    }

    private void runSubMode(SubMode subMode) {
        switch (subMode) {
            case MAIN:
                runMain();
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

    private void runMain() {
        telemetry.addData("Running", "MAIN");
        telemetry.update();
    }
}