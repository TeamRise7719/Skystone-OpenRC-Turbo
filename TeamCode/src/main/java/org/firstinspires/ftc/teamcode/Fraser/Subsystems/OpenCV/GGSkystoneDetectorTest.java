package org.firstinspires.ftc.teamcode.Fraser.Subsystems.OpenCV;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name="GG Detector Test")
public class GGSkystoneDetectorTest extends LinearOpMode {

    GGSkystoneDetector vis;
    GGOpenCV detector;
    boolean camFound;
    @Override

    public void runOpMode(){

        GGOpenCV detector = new GGOpenCV(hardwareMap);
        detector.startLook(VisionSystem.TargetType.SKYSTONE);

        while (!this.isStarted()){

            if (detector.found()){
                telemetry.addData("SS Found!", "");
                telemetry.addData("X: ", detector.detector.foundRectangle().x);
                telemetry.addData("Y: ", detector.detector.foundRectangle().y);
            } else {
                telemetry.addData("SS not found.", "");
            }
            telemetry.update();

        }
        requestOpModeStop();

    }

}
