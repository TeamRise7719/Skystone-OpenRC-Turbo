//package org.firstinspires.ftc.teamcode.SeansSpace.EasyOpenCV;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//import org.firstinspires.ftc.robotcore.external.ClassFactory;
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
//import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
//import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
//import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
//import org.opencv.core.Mat;
//import org.opencv.core.Point;
//import org.opencv.core.Scalar;
//import org.opencv.imgproc.Imgproc;
//import org.openftc.easyopencv.OpenCvCameraFactory;
//import org.openftc.easyopencv.OpenCvCameraRotation;
//import org.openftc.easyopencv.OpenCvInternalCamera;
//import org.openftc.easyopencv.OpenCvPipeline;
//
//import java.util.ArrayList;
//import java.util.List;
//
///**
// * Created by Sean Cardosi on 2019-12-03.
// */
//@TeleOp(name = "Vuforia With Zooming",group = "EasyOpenCV")
//public class VuforiaZoom extends LinearOpMode {
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//
//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        int[] viewportContainerIds = OpenCvCameraFactory.getInstance().splitLayoutForMultipleViewports(cameraMonitorViewId, 2, OpenCvCameraFactory.ViewportSplitMethod.VERTICALLY);
//
//        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(viewportContainerIds[0]);
//        parameters.vuforiaLicenseKey = "AUbWH2r/////AAABmTqIGJpUgkb9j4jexPb0CKYmpPjDhVyV5bE2RL866jD2AKG/pqN0P8mPlybMo3P0xERT+mK0uW04FHPso8OcJDLER7gW6Rjnv49Yzc7ks3zkCGtwmHx/sqInqUl4i2jlHTFiW8qYnAf/iOJ0O2jO7j8UjOuurbpGT+3iGwRprWQFe7/Wb6k08A1tMIwvDKgU3g+PudWyfefPeo2Oo3PYzIiGu+KlswOR26Jn3jRSGmlin3JrfLkvmV7AmTaFWGwb876eR21A5EP40EIBk8E9nDuJcVB60q9R7nnBvf/qjsSuUwKQWtn9xTBzWSIhEzXetUY5PMxckiugQLWHp7YAkrM7SGNte2JrUk9XslYRqV4z";
//        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK; //required for webcam
//        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
//        VuforiaLocalizer vuforia = ClassFactory.getInstance().createVuforia(parameters);
//
//        VuforiaTrackables targetsSkyStone = vuforia.loadTrackablesFromAsset("Skystone");
//
//        VuforiaTrackable stoneTarget = targetsSkyStone.get(0);
//        stoneTarget.setName("Stone Target");
//        VuforiaTrackable blueRearBridge = targetsSkyStone.get(1);
//        blueRearBridge.setName("Blue Rear Bridge");
//        VuforiaTrackable redRearBridge = targetsSkyStone.get(2);
//        redRearBridge.setName("Red Rear Bridge");
//        VuforiaTrackable redFrontBridge = targetsSkyStone.get(3);
//        redFrontBridge.setName("Red Front Bridge");
//        VuforiaTrackable blueFrontBridge = targetsSkyStone.get(4);
//        blueFrontBridge.setName("Blue Front Bridge");
//        VuforiaTrackable red1 = targetsSkyStone.get(5);
//        red1.setName("Red Perimeter 1");
//        VuforiaTrackable red2 = targetsSkyStone.get(6);
//        red2.setName("Red Perimeter 2");
//        VuforiaTrackable front1 = targetsSkyStone.get(7);
//        front1.setName("Front Perimeter 1");
//        VuforiaTrackable front2 = targetsSkyStone.get(8);
//        front2.setName("Front Perimeter 2");
//        VuforiaTrackable blue1 = targetsSkyStone.get(9);
//        blue1.setName("Blue Perimeter 1");
//        VuforiaTrackable blue2 = targetsSkyStone.get(10);
//        blue2.setName("Blue Perimeter 2");
//        VuforiaTrackable rear1 = targetsSkyStone.get(11);
//        rear1.setName("Rear Perimeter 1");
//        VuforiaTrackable rear2 = targetsSkyStone.get(12);
//        rear2.setName("Rear Perimeter 2");
//
//        List<VuforiaTrackable> allTrackables = new ArrayList<>(targetsSkyStone);
//
//        targetsSkyStone.activate();
//
//
//        OpenCvInternalCamera phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, viewportContainerIds[1]);
//        phoneCam.openCameraDevice();
//        phoneCam.setPipeline(new SamplePipeline());
//        phoneCam.startStreaming(320, 240, OpenCvCameraRotation.SIDEWAYS_RIGHT);
//
//        int zoom = 0;
//
//        waitForStart();
//
//        while (opModeIsActive()) {
//
//            phoneCam.setZoom(zoom);
//
//            if (gamepad1.y) {
//                if (zoom < phoneCam.getMaxSupportedZoom()) {
//                    zoom++;
//                }
//            } else if (gamepad1.a) {
//                if (zoom > 0) {
//                    zoom--;
//                }
//            }
//            for (VuforiaTrackable trackable : allTrackables) {
//                if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
//                    telemetry.addData("Visible Target", trackable.getName());
//                }
//            }
//
//            telemetry.addData("Internal cam FPS", phoneCam.getFps());
//            telemetry.addData("Current Zoom",zoom);
//            telemetry.addData("Controls"," y to increase zoom and a to decrease zoom");
//            telemetry.update();
//
//            sleep(100);
//        }
//    }
//
//    class SamplePipeline extends OpenCvPipeline {
//        @Override
//        public Mat processFrame(Mat input) {
//            Imgproc.rectangle(
//                    input,
//                    new Point(
//                            input.cols() / 4,
//                            input.rows() / 4),
//                    new Point(
//                            input.cols() * (3f / 4f),
//                            input.rows() * (3f / 4f)),
//                    new Scalar(0, 255, 0), 4);
//
//            return input;
//        }
//    }
//}
