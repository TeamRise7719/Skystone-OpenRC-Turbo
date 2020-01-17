//package org.firstinspires.ftc.teamcode.Fraser.FraserModes;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.teamcode.Fraser.Subsystems.Drive.FraserComponents;
//import org.firstinspires.ftc.teamcode.Fraser.Subsystems.Drive.SeansEncLibrary;
//import org.firstinspires.ftc.teamcode.Fraser.Subsystems.Media.RobotMedia;
//import org.firstinspires.ftc.teamcode.Fraser.Subsystems.OpenCV.GGOpenCV;
//import org.firstinspires.ftc.teamcode.Fraser.Subsystems.OpenCV.VisionSystem;
//import org.firstinspires.ftc.teamcode.Fraser.Subsystems.Transitioning.AutoTransitioner;
//
///**
// * Created by Sean Cardosi on 2020-01-09.
// */
//public abstract class FraserLinearOpMode extends LinearOpMode {
//
//    public SeansEncLibrary enc;
//    public FraserComponents component;
//    public RobotMedia media;
//    public GGOpenCV detector;
//    public ElapsedTime etime = new ElapsedTime();
//
//    public FraserLinearOpMode() {}
//
//    public final void waitFor(int time){
//        time = time/1000;
//        etime.reset();
//        while ((etime.time() < time)&&(opModeIsActive())) {
//            idle();
//        }
//    }
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//
//        enc = new SeansEncLibrary(hardwareMap,telemetry,this);
//        enc.init();
//        component = new FraserComponents(hardwareMap);
//        component.init();
//        media = new RobotMedia(hardwareMap);
//
//        waitForStart();
//
//        media.startTimer();
//    }
//
//    @Override
//    public synchronized void waitForStart() {
//        super.waitForStart();
//    }
//
//    public final void skystoneScanProcess(double position) {
//        detector = new GGOpenCV(GGOpenCV.Cam.PHONE, hardwareMap);
//        detector.startCamera();
//        detector.startLook(VisionSystem.TargetType.SKYSTONE);
//        while (!this.isStarted){
//            if (detector.found()){
//                telemetry.addData("Skystone Found!", "");
//                telemetry.addData("X: ", detector.detector.foundRectangle().x);
//                telemetry.addData("Y: ", detector.detector.foundRectangle().y);
//
//                position = detector.detector.foundRectangle().x;
//                telemetry.addData("Position: ", position);
//
//            } else {
//                telemetry.addData("Skystone not found.", "");
//            }
//            telemetry.update();
//        }
//        detector.stopLook();
//        media.startTimer();
//    }
//
//
//    public final void toTeleOp() {
//        AutoTransitioner.transitionOnStop(this,"Fraser TeleOp");
//    }
//
//}
