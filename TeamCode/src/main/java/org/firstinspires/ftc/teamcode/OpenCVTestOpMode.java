package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.widget.FrameLayout;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import ftc.team6460.javadeck.ftc.vision.*;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Mat;
@Autonomous(name = "OpenCV Test")
public class OpenCVTestOpMode extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        try {
            telemetry.addData(":(", ":)");
            waitForStart();
            Activity activity = (Activity) (this.hardwareMap.appContext);
            FrameLayout pv = (FrameLayout) activity.findViewById(R.id.previewLayout);
            final OpenCvActivityHelper ocvh = new OpenCvActivityHelper(activity, pv);
            ocvh.addCallback(new MatCallback() {
                @Override
                public void handleMat(Mat mat) {
                    // pass
                }

                @Override
                public void draw(Canvas canvas) {
                    Paint p = new Paint();
                    p.setColor(Color.RED);
                    canvas.drawLine(-20, -20, 20, 20, p);
                }
            });
            ocvh.attach();
            ocvh.awaitStart();
            int loops = 0;
            while (opModeIsActive()) {
                loops++;
                telemetry.addData("cvLoops", loops);
            }
            ocvh.stop();
        } catch (Exception e){
            int a = 2;
            int b = 1+a;
            telemetry.addData(":(", ":(");
            telemetry.addData(":/", e.getMessage());
            telemetry.addData(":/", e.getClass().getName());
        }
    }
}
