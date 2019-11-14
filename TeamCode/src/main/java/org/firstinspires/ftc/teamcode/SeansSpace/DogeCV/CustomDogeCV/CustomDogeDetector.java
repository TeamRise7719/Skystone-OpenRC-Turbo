package org.firstinspires.ftc.teamcode.SeansSpace.DogeCV.CustomDogeCV;

import com.disnodeteam.dogecv.detectors.skystone.SkystoneDetector;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.SeansSpace.DogeCV.CustomDogeCV.DogeMath.DogeVector;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvWebcam;

/**
 * @Author Sean Cardosi
 * @Date 11/14/19
 */
public class CustomDogeDetector {
    private OpenCvCamera phoneCamera;
    private OpenCvWebcam webcam;
    public SkystoneDetector skystoneDetector;
    private Cam cam;

    public enum Cam{
        PHONE, WEBCAM
    }

    public CustomDogeDetector(Cam cam, HardwareMap hardwareMap){
        this.cam = cam;
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        if(cam.equals(Cam.PHONE)){
            phoneCamera = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.FRONT, cameraMonitorViewId);
            skystoneDetector = new SkystoneDetector();
            phoneCamera.setPipeline(skystoneDetector);
        }
        else if(cam.equals(Cam.WEBCAM)){
            webcam = new OpenCvWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
            skystoneDetector = new SkystoneDetector();
            webcam.setPipeline(skystoneDetector);
        }
    }

    public void start(){
        if(cam.equals(Cam.PHONE)){
            phoneCamera.openCameraDevice();
            phoneCamera.startStreaming(320,240, OpenCvCameraRotation.UPRIGHT);
        }
        else if(cam.equals(Cam.WEBCAM)){
            webcam.openCameraDevice();
            webcam.startStreaming(320,240, OpenCvCameraRotation.UPRIGHT);
        }
    }

    public void stop(){
        if(cam.equals(Cam.PHONE)){
            phoneCamera.stopStreaming();
            phoneCamera.closeCameraDevice();
        }
        else if(cam.equals(Cam.WEBCAM)){
            webcam.stopStreaming();
            webcam.closeCameraDevice();
        }
    }

    public void pauseViewPort(){
        if(cam.equals(Cam.PHONE)) phoneCamera.pauseViewport();
        else if(cam.equals(Cam.WEBCAM)) webcam.pauseViewport();
    }

    public void resumeViewPort(){
        if(cam.equals(Cam.PHONE)) phoneCamera.resumeViewport();
        else if(cam.equals(Cam.WEBCAM)) webcam.resumeViewport();
    }

    public SkystonePosition getStoneSkystonePosition(){
        double x = getStoneX();
        if(x < 160) return SkystonePosition.LEFT;
        else if(x >= 160 && x <= 320) return SkystonePosition.MIDDLE;
        else if(x > 320) return SkystonePosition.RIGHT;
        return null;
    }

    public DogeVector getStoneSkystoneVector(){
        SkystonePosition position = getStoneSkystonePosition();
        if(position.equals(SkystonePosition.LEFT)) return new DogeVector(-8,29);
        else if(position.equals(SkystonePosition.MIDDLE)) return new DogeVector(0,20);
        else if(position.equals(SkystonePosition.RIGHT)) return new DogeVector(8,29);
        return null;
    }

    public double getStoneX(){
        return skystoneDetector.getScreenPosition().x;
    }

    public double getStoneY(){
        return skystoneDetector.getScreenPosition().y;
    }

    public boolean isDetected(){
        return skystoneDetector.isDetected();
    }


    public enum SkystonePosition {
        LEFT, MIDDLE, RIGHT
    }
}