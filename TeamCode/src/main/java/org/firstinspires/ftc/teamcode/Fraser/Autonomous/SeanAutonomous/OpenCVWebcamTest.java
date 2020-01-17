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
    public void runOpMode() {

        detector = new GGOpenCVWebcam(telemetry,hardwareMap,this);
        detector.startLook(VisionSystem.TargetType.SKYSTONE);

        detector.scanInit();

        waitForStart();

        detector.scanMain();

    }
}
