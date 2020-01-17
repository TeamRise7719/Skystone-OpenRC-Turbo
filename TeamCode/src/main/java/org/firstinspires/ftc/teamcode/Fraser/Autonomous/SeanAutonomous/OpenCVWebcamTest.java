package org.firstinspires.ftc.teamcode.Fraser.Autonomous.SeanAutonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Fraser.Subsystems.OpenCV.GGOpenCVWebcam;
import org.firstinspires.ftc.teamcode.Fraser.Subsystems.OpenCV.VisionSystem;

/**
 * Created by Sean Cardosi on 2020-01-15.
 */
@Autonomous(name = "OpenCVWebcamTest", group = "OpenCV")
public class OpenCVWebcamTest extends LinearOpMode {

    GGOpenCVWebcam detector;

    @Override
    public void runOpMode() throws InterruptedException {

        try {

            detector = new GGOpenCVWebcam(GGOpenCVWebcam.Cam.WEBCAM, hardwareMap);

        } catch (IllegalArgumentException e) {
            e.printStackTrace();
            requestOpModeStop();
        }

        waitForStart();

        detector.startCamera();
        detector.startLook(VisionSystem.TargetType.SKYSTONE);

        while (opModeIsActive()) {
            if (detector.found()) {
                telemetry.addData("Skystone Found!", "");
                telemetry.addData("X: ", detector.detector.foundRectangle().x);
                telemetry.addData("Y: ", detector.detector.foundRectangle().y);
                double x = detector.detector.foundRectangle().x;
                double y = detector.detector.foundRectangle().y;
                telemetry.addData("(X,Y)","%f,%f",x,y);
                telemetry.addData("Position (X): ", detector.detector.foundRectangle().x);
            } else {
                telemetry.addData("Skystone not found.", "");
            }
            telemetry.update();
        }
        detector.stopLook();
    }
}
