package org.firstinspires.ftc.teamcode.Fraser.Autonomous.JordanAutonomous.FrogBot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Fraser.Subsystems.Drive.FraserComponents;
import org.firstinspires.ftc.teamcode.Fraser.Subsystems.Drive.SeansEncLibrary;
import org.firstinspires.ftc.teamcode.Fraser.Subsystems.OpenCV.GGOpenCV;
import org.firstinspires.ftc.teamcode.Fraser.Subsystems.OpenCV.VisionSystem;

@Autonomous (name = "Blue Frog Bot", group = "Fraser Blue Auto")
public class Blue_Frog_Bot extends LinearOpMode {
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

            } else {
                telemetry.addData("SS not found.", "");
            }
            telemetry.update();

        }
        detector.stopLook();

//      position 3 and 0
        if ((posit >= 0 && posit <= 57) || (posit >= 240 && posit < 280)) {

            mech.clawRelease();
            enc.steeringDrive(24, false, false);
            enc.arcTurn(-89);
            enc.steeringDrive(-4, false, false);
            mech.intakeStone();
            enc.steeringDrive(-19, false, true);
            enc.steeringDrive(9, false, false);
            waitFor(500);
            mech.clawGrab();
            mech.stopStone();
            enc.steeringDrive(19.7, false, true);
//            enc.steeringDrive(-82, false, false);
//            enc.arcTurn(-90);
//            enc.steeringDrive(6, false, true);
//            enc.steeringDrive(-12, false, false);
////            waitFor(500);
//            mech.shoulderUp();
//            waitFor(1500);
//            mech.autoClaw();
//            waitFor(500);
//            mech.shoulderDown();
//            mech.foundationRelease();
//            waitFor(1500);
//            enc.steeringDrive(28, false, false);
//            enc.arcTurn(90);
//            mech.foundationGrab();
//            enc.arcTurn(-55);
//            enc.steeringDrive(-15, false, true);
//            enc.steeringDrive(-23, false, false);
//            enc.steeringDrive(-23, false, true);


        }

        //  position 1 and 4
        if ((posit >= 60 && posit <= 100)) {

            mech.clawRelease();
            enc.steeringDrive(10, false, false);
            enc.arcTurn(-90);
            enc.steeringDrive(6, false, false);
            mech.intakeStone();
            enc.steeringDrive(-33.8, false, true);
            enc.steeringDrive(9, false, false);
            waitFor(500);
            mech.clawGrab();
            mech.stopStone();
            enc.steeringDrive(18, false, true);
            enc.steeringDrive(-91, false, false);
            enc.arcTurn(-90);
            enc.steeringDrive(4, false, true);
            enc.steeringDrive(-9.5, false, false);
            waitFor(500);
            mech.shoulderUp();
            waitFor(1500);
            mech.autoClaw();
            waitFor(500);
            mech.shoulderDown();
            mech.foundationRelease();
            waitFor(1500);
            enc.steeringDrive(28, false, false);
            enc.arcTurn(90);
            mech.foundationGrab();
            enc.arcTurn(-65);
            enc.steeringDrive(-15, false, true);
            enc.steeringDrive(-21, false, false);
            enc.steeringDrive(-23, false, true);


        }
//                   position 2 and 5
        if ((posit >= 130 && posit <= 190)) {

            mech.clawRelease();
            enc.steeringDrive(10, false, false);
            enc.arcTurn(-91);
            enc.steeringDrive(15, false, false);
            mech.intakeStone();
            enc.steeringDrive(-33.8, false, true);
            enc.steeringDrive(6.5, false, false);
            waitFor(500);
            mech.clawGrab();
            mech.stopStone();
            enc.steeringDrive(14, false, true);
            enc.steeringDrive(-91, false, false);
            enc.arcTurn(-90);
            enc.steeringDrive(11, false, true);
            enc.steeringDrive(-8.5, false, false);
            waitFor(500);
            mech.shoulderUp();
            waitFor(1500);
            mech.autoClaw();
            waitFor(500);
            mech.shoulderDown();
            mech.foundationRelease();
            waitFor(1500);
            enc.steeringDrive(28, false, false);
            enc.arcTurn(90);
            mech.foundationGrab();
            enc.arcTurn(-45);
            enc.steeringDrive(-15, false, true);
            enc.steeringDrive(-24, false, false);
            enc.steeringDrive(-23, false, true);


        }


    }
}
