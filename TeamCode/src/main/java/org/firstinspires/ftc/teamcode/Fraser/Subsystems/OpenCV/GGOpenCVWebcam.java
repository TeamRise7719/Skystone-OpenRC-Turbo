package org.firstinspires.ftc.teamcode.Fraser.Subsystems.OpenCV;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Rect;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

/**
 * Created by Sean Cardosi on 2020-01-15.
 */
public class GGOpenCVWebcam implements VisionSystem {

    public static final Rect CAMERA_RECT = new Rect(0, 0, 320, 240);

    public OpenCvCamera camera;
    public GGSkystoneDetector detector;
    HardwareMap hardwareMap;
    WebcamName webcam;
    Cam cam;

    @Override
    public void startLook(VisionSystem.TargetType targetType) {
        switch (targetType) {
            case SKYSTONE: {
                detector.useDefaults();
                break;
            }
        }
        startCamera();
    }

    @Override
    public void stopLook() {
        camera.stopStreaming();
        camera.closeCameraDevice();
    }

    public enum Cam {
        PHONE, WEBCAM
    }

    @Override
    public boolean found() {
        return detector.isDetected();
    }

    public GGOpenCVWebcam(Cam cam, HardwareMap hardwareMap) {
        this.cam = cam;
        detector = new GGSkystoneDetector();
        webcam = hardwareMap.get(WebcamName.class, "webcam");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = new OpenCvWebcam(webcam, cameraMonitorViewId);
        camera.openCameraDevice();
    }

    public void startCamera() {
        camera.setPipeline(detector);
        camera.startStreaming(CAMERA_RECT.width, CAMERA_RECT.height, OpenCvCameraRotation.SIDEWAYS_LEFT);
    }

}
