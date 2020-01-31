package org.firstinspires.ftc.teamcode.Fraser.Subsystems.OpenCV;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Rect;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvWebcam;

public class GGOpenCV implements VisionSystem {

    private static final Rect CAMERA_RECT = new Rect(0, 0, 320, 240);

    public Cam cam;
    private OpenCvWebcam camera;
    public GGSkystoneDetector detector;
    HardwareMap hardwareMap;

    public enum Cam{
        PHONE, WEBCAM
    }

    @Override
    public void startLook(TargetType targetType) {
        switch (targetType) {
            case SKYSTONE: {
                detector = new GGSkystoneDetector();
                detector.useDefaults();
                break;
            }
            default: {
                detector = new GGSkystoneDetector();
                detector.useDefaults();
            }
        }
        startCamera();
    }

    @Override
    public void stopLook() {
        camera.stopStreaming();
    }

    @Override
    public boolean found() {
        return detector.isDetected();
    }

    public GGOpenCV(HardwareMap hardwareMap) {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = new OpenCvWebcam(hardwareMap.get(WebcamName.class, "webcam"), cameraMonitorViewId);
        camera.openCameraDevice();
    }

    public void startCamera() {
        camera.setPipeline(detector);
        camera.startStreaming(CAMERA_RECT.width, CAMERA_RECT.height, OpenCvCameraRotation.SIDEWAYS_RIGHT);
    }

}