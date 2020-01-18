package org.firstinspires.ftc.teamcode.Fraser.Subsystems.OpenCV;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Fraser.Subsystems.NewOpenCV.OpenCvCameraFactory;
import org.opencv.core.Rect;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

/**
 * Created by Sean Cardosi on 2020-01-15.
 */
public class GGOpenCVWebcam implements VisionSystem {

    public static final Rect CAMERA_RECT = new Rect(0, 0, 640, 480);

    public OpenCvCamera camera;
    public GGSkystoneDetector detector;
    HardwareMap hardwareMap;
    LinearOpMode linearOpMode;
    Telemetry telemetry;

    @Override
    public void startLook(VisionSystem.TargetType targetType) {
        detector = new GGSkystoneDetector();
        detector.useDefaults();
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "webcam"), cameraMonitorViewId);
        camera.openCameraDevice();
        camera.setPipeline(detector);
        camera.startStreaming(CAMERA_RECT.width, CAMERA_RECT.height, OpenCvCameraRotation.UPRIGHT);
    }

    @Override
    public void stopLook() {
        camera.stopStreaming();
        camera.closeCameraDevice();
    }

    @Override
    public boolean found() {
        return detector.isDetected();
    }

    public GGOpenCVWebcam(Telemetry tel, HardwareMap hardwareMap, LinearOpMode opMode) {
        detector = new GGSkystoneDetector();
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "webcam"), cameraMonitorViewId);
        linearOpMode = opMode;
        telemetry = tel;
    }

    public void scan() {
        startLook(TargetType.SKYSTONE);

        while (!linearOpMode.isStarted()) {
            telemetry.addData("(x,y)", "%f,%f", detector.foundRectangle().x, detector.foundRectangle().y);
            telemetry.addData("Position (x): ", detector.foundRectangle().x);
            telemetry.update();
            linearOpMode.sleep(100);
        }

        stopLook();
    }
}
