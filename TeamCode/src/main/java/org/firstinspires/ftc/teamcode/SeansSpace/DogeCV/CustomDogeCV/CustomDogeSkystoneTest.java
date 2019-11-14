package org.firstinspires.ftc.teamcode.SeansSpace.DogeCV.CustomDogeCV;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

/**
 * @Author Sean Cardosi
 * @Date 11/14/19
 */
@Autonomous(name = "Custom Doge Skystone Test", group = "DogeCV")
public class CustomDogeSkystoneTest extends OpMode {

    CustomDogeDetector detector;
    CustomDogeDetector.SkystonePosition skystonePosition;
    @Override
    public void init() {
        detector.start();
        telemetry.addData("Stone Detected: ", detector.isDetected());
    }
        public void loop() {

        if (!detector.isDetected()) {
            telemetry.addData("SkystoneStatus", detector.isDetected());
        } else {
            skystonePosition = detector.getStoneSkystonePosition();

            if (skystonePosition == null) {
                telemetry.addData("Skystone Position: ", "UNKNOWN");
            } else if (skystonePosition == CustomDogeDetector.SkystonePosition.LEFT) {
                telemetry.addData("Skystone Position: ", "LEFT");
            } else if (skystonePosition == CustomDogeDetector.SkystonePosition.RIGHT) {
                telemetry.addData("Skystone Position: ", "RIGHT");
            } else if (skystonePosition == CustomDogeDetector.SkystonePosition.MIDDLE) {
                telemetry.addData("Skystone Position: ", "LEFT");
            }
        }
    }
    public void stop() {
        detector.stop();
    }
}