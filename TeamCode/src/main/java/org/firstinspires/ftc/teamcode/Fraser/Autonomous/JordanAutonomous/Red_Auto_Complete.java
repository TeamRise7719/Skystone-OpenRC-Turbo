package org.firstinspires.ftc.teamcode.Fraser.Autonomous.JordanAutonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Fraser.Subsystems.Drive.FraserComponents;
import org.firstinspires.ftc.teamcode.Fraser.Subsystems.Drive.SeansEncLibrary;
import org.firstinspires.ftc.teamcode.Fraser.Subsystems.OpenCV.GGOpenCV;
import org.firstinspires.ftc.teamcode.Fraser.Subsystems.OpenCV.VisionSystem;

@Autonomous (name = "Red Auto Complete " , group = "Fraser Red Auto" )
public class Red_Auto_Complete extends LinearOpMode{

    ElapsedTime etime = new ElapsedTime();
    private double posit = 0;

    public void waitFor(int time){
        time = time/1000;
        etime.reset();
        while ((etime.time() < time)&&(opModeIsActive())) {
            idle();
        }
    }

    @Override
    public void runOpMode(){

        SeansEncLibrary enc = new SeansEncLibrary(hardwareMap, telemetry, this);
        FraserComponents mech = new FraserComponents(hardwareMap);
//        GGOpenCV detector = new GGOpenCV(GGOpenCV.Cam.PHONE, hardwareMap);
//        detector.startCamera();
//        detector.startLook(VisionSystem.TargetType.SKYSTONE);

        enc.init();
        mech.init();

        waitForStart();



//        while (!this.isStarted){
//            if (detector.found()){
//                telemetry.addData("SS Found!", "");
//                telemetry.addData("X: ", detector.detector.foundRectangle().x);
//                telemetry.addData("Y: ", detector.detector.foundRectangle().y);
//
//                posit = detector.detector.foundRectangle().x;
//                telemetry.addData("Position: ", posit);
//
//            } else {
//                telemetry.addData("SS not found.", "");
//            }
//
//            telemetry.update();
//        }
//
//        detector.stopLook();

//      position 3 and 0
//        if((posit>=225&&posit<=290)||(posit>=35&&posit<65)){
//
//
//            enc.steeringDrive(24,false,true);
//            enc.steeringDrive(-4,false,false);
//            enc.steeringDrive(20, false, true);
//            waitFor(1000);
//            mech.intakeStone();
//            waitFor(500);
//            enc.steeringDrive(4,false,false);
//            waitFor(1000);
//            mech.stopStone();
//            waitFor(500);
//            enc.steeringDrive(-12,false,true);
//            enc.steeringDrive(-60,false,false);
//
//
//
//        }

        //     position 1 and 4
//        if ((posit>=170&&posit<=220)||(posit>=1&&posit<=30)){

//        mech.ejectStone();
//               enc.steeringDrive(6,false,false);
//        enc.steeringDrive(23,false,true);
//        enc.arcTurn(92);
//        mech.intakeStone();
//        enc.steeringDrive( 40.2, false, true);
////        enc.steeringDrive(1.5,false,false);
////        waitFor(750);
//
//        enc.steeringDrive(6, false, false);
//
//        enc.steeringDrive(-22,false,true);
//        mech.stopStone();
//        enc.steeringDrive(-45,false,false);
//        enc.arcTurn(276);
//        mech.ejectStone();
//        waitFor(500);
//        enc.arcTurn(93);
//        mech.stopStone();
//        enc.steeringDrive(62,false,false);
//        enc.steeringDrive(18.7,false, true);
//        mech.intakeStone();
//        waitFor(500);
//        enc.steeringDrive(6, false, false);
//        mech.stopStone();
//        enc.steeringDrive(-18,false, true);
//        enc.steeringDrive(-63,false,false);
//        enc.arcTurn(185);
//        mech.ejectStone();
//        waitFor(500);
//        enc.steeringDrive(-10,false,false);
//



//        }
//         position 2 and 5
//        if ((posit>=110&&posit<=169)||(posit==0)){

//            mech.ejectStone();
            enc.steeringDrive(6,false, false);
            enc.steeringDrive(13.5, false,true);
            enc.arcTurn(92);
            mech.intakeStone();
            enc.steeringDrive(39.2,false,true);
            enc.steeringDrive(4.5,false,false);
            enc.steeringDrive(-19,false,true);
            mech.stopStone();
            enc.steeringDrive(-50,false,false);
            enc.arcTurn(278);
            mech.ejectStone();
            waitFor(1000);
            enc.arcTurn(93);
            mech.stopStone();
            enc.steeringDrive(69,false,false);
            enc.steeringDrive(20,false, true);
            mech.intakeStone();
            waitFor(500);
            enc.steeringDrive(6, false, false);
            mech.stopStone();
            enc.steeringDrive(-18,false, true);
            enc.steeringDrive(-72,false,false);
            enc.arcTurn(185);
            mech.ejectStone();
            waitFor(500);
            enc.steeringDrive(-10,false,false);

//
//        }


    }



}
