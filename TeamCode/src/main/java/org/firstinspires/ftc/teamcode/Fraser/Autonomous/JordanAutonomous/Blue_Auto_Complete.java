package org.firstinspires.ftc.teamcode.Fraser.Autonomous.JordanAutonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Fraser.Subsystems.Drive.FraserComponents;
import org.firstinspires.ftc.teamcode.Fraser.Subsystems.Drive.SeansEncLibrary;
import org.firstinspires.ftc.teamcode.Fraser.Subsystems.OpenCV.GGOpenCV;
import org.firstinspires.ftc.teamcode.Fraser.Subsystems.OpenCV.VisionSystem;

@Autonomous (name = "Blue Auto Complete", group = "Fraser Blue Auto")
public class Blue_Auto_Complete extends LinearOpMode {
    GGOpenCV detector;
    private ElapsedTime etime = new ElapsedTime();
    private double posit = 0;

    private void waitFor(int time) {
        time = time / 1000;
        etime.reset();
        while ((etime.time() < time) && (opModeIsActive())) {
            idle();
        }
    }

    @Override
    public void runOpMode() {

        SeansEncLibrary enc = new SeansEncLibrary(hardwareMap, telemetry, this);
        FraserComponents mech = new FraserComponents(hardwareMap);

        enc.init();
        mech.init();

        GGOpenCV detector = new GGOpenCV(hardwareMap);
        detector.startLook(VisionSystem.TargetType.SKYSTONE);

        while (!this.isStarted()) {

            if (detector.found()) {
                telemetry.addData("SS Found!", "");
                telemetry.addData("X: ", detector.detector.foundRectangle().x);
                telemetry.addData("Y: ", detector.detector.foundRectangle().y);
                posit = detector.detector.foundRectangle().y;
                detector.stopLook();
            } else {
                telemetry.addData("SS not found.", "");
            }
            telemetry.update();

        }

        waitForStart();

//      position 3 and 0
        if ((posit >= 0 && posit <= 57) || (posit >= 246 && posit < 280)) {

            mech.clawRelease();
            enc.steeringDrive(24, false, false);
            enc.arcTurn(89);
            enc.steeringDrive(-4, false, false);
            mech.intakeStone();
            enc.steeringDrive(19, false, true);
            enc.steeringDrive(7, false, false);
            waitFor(500);
            mech.clawGrab();
            mech.stopStone();
            enc.steeringDrive(-17.7, false, true);
            enc.steeringDrive(-80, false, false);
            enc.arcTurn(90);
            enc.steeringDrive(-6, false, true);
            enc.steeringDrive(-8, false, false);
//            waitFor(500);
            mech.shoulderUp();
            waitFor(1500);
            mech.autoClaw();
            waitFor(500);
            mech.shoulderDown();
            mech.foundationRelease();
            waitFor(1500);
            enc.steeringDrive(32,false,false);
            enc.arcTurn(-90);
            mech.foundationGrab();
            enc.arcTurn(45);
            enc.steeringDrive(15,false,true);
            enc.steeringDrive(-25,false,false);
            enc.steeringDrive(23,false,true);




        }

        //  position 1 and 4
        if ((posit >= 60 && posit <= 100)) {

            mech.clawRelease();
            enc.steeringDrive(10,false,false);
            enc.arcTurn(-90);
            //enc.arcTurn(-93);
            enc.steeringDrive(4,false,false);
            mech.intakeStone();
            enc.steeringDrive( -34.8, false, true);
            enc.steeringDrive(4.5, false, false);
            waitFor(500);
            mech.clawGrab();
            mech.stopStone();
            enc.steeringDrive(18,false,true);
            enc.steeringDrive(-90,false,false);
            enc.arcTurn(-90);
            enc.steeringDrive(4,false,true);
            //enc.steeringDrive(-11.5,false,false);
            enc.steeringDrive(-7.5,false,false);
            waitFor(500);
            mech.shoulderUp();
            waitFor(1500);
            mech.autoClaw();
            waitFor(500);
            mech.shoulderDown();
            mech.foundationRelease();
            waitFor(1500);
            enc.steeringDrive(32,false,false);
            enc.arcTurn(90);
            mech.foundationGrab();
            enc.arcTurn(-45);
            enc.steeringDrive(-15,false,true);
            enc.steeringDrive(-25,false,false);
            enc.steeringDrive(-23,false,true);


        }
//                   position 2 and 5
        if ((posit >= 150 && posit <= 170)) {

            mech.clawRelease();
            enc.steeringDrive(10,false,false);
            enc.arcTurn(90);
            enc.steeringDrive(15,false,false);
            mech.intakeStone();
            enc.steeringDrive( 34.8, false, true);
            enc.steeringDrive(4.5, false, false);
            waitFor(500);
            mech.clawGrab();
            mech.stopStone();
            enc.steeringDrive(-18,false,true);
            enc.steeringDrive(-95,false,false);
            enc.arcTurn(90);
            enc.steeringDrive(-4,false,true);
            enc.steeringDrive(-7.5,false,false);
            waitFor(500);
            mech.shoulderUp();
            waitFor(1500);
            mech.autoClaw();
            waitFor(500);
            mech.shoulderDown();
            mech.foundationRelease();
            waitFor(1500);
            enc.steeringDrive(32,false,false);
            enc.arcTurn(-90);
            mech.foundationGrab();
            enc.arcTurn(45);
            enc.steeringDrive(15,false,true);
            enc.steeringDrive(-25,false,false);
            enc.steeringDrive(23,false,true);


        }


    }
    }
