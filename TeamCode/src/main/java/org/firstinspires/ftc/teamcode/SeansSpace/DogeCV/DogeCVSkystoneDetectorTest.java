package org.firstinspires.ftc.teamcode.SeansSpace.DogeCV;

import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.skystone.SkystoneDetector;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.opencv.core.Rect;

@TeleOp(name = "DogeCV Skystone Detector Test", group = "DogeCV")
public class DogeCVSkystoneDetectorTest extends OpMode {

    SkystoneDetector detector;

    @Override
    public void init() {

        telemetry.addData("Status: ", "DogeCV 2020 has Successfully Initialized");

        detector = new SkystoneDetector();
        detector.useDefaults();
        detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA;
        detector.maxAreaScorer.weight = 0.01;
        detector.ratioScorer.weight = 3;
        detector.ratioScorer.perfectRatio = 1.25;
    }

    @Override
    public void loop() {
        telemetry.addData("IsFound: ", detector.isDetected());
        Rect rect = detector.foundRectangle();
        if (detector.isDetected()) {
            telemetry.addData("Location: ", Integer.toString((int) (rect.x + rect.width*0.5)) + ", " + Integer.toString((int) (rect.y+0.5*rect.height)));
            telemetry.update();
        }
    }
}
