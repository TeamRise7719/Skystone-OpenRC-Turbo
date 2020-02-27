package org.firstinspires.ftc.teamcode.Fraser.Autonomous.JordanAutonomous.Regular;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Fraser.Subsystems.Drive.FraserComponents;
import org.firstinspires.ftc.teamcode.Fraser.Subsystems.Drive.SeansEncLibrary;
import org.firstinspires.ftc.teamcode.Fraser.Subsystems.OpenCV.GGOpenCV;
import org.firstinspires.ftc.teamcode.Fraser.Subsystems.OpenCV.VisionSystem;

@Autonomous (name = "Red Auto Complete " , group = "Fraser Red Auto" )
public class Red_Auto_Complete extends LinearOpMode {

    private ElapsedTime etime = new ElapsedTime();
    private double posit = 0;

    private void waitFor(int time) {
        time = time / 1000;
        etime.reset();
        while ((etime.time() < time) && (opModeIsActive())) {
            idle();
        }
    }


    /**
     * Dedicated to Kobe Bryant  #24
     */


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
        if ((posit >= 200 && posit <= 260) || (posit >= 0 && posit < 50)) {

            mech.ejectStone();
            mech.clawRelease();
            enc.steeringDrive(22, false, false);
            mech.stopStone();
            enc.arcTurn(92);
            enc.steeringDrive(-4, false, false);
            waitFor(100);
            mech.intakeStone();
            enc.steeringDrive(19, false, true);
            waitFor(125);
            enc.steeringDrive(6.5, false, false);
            mech.clawGrab();
            mech.stopStone();
            enc.arcTurn(-2);
            enc.steeringDrive(-19, false, true);
            enc.steeringDrive(-80, false, false);
            enc.arcTurn(90);
            enc.steeringDrive(-7, false, true);
            enc.steeringDrive(-10, false, false);
            mech.shoulderUp();
            waitFor(1500);
            mech.autoClaw();
            mech.shoulderDown();
            mech.foundationRelease();
            waitFor(1500);
            enc.steeringDrive(28, false, false);
            enc.arcTurn(-90);
            mech.foundationGrab();
            enc.arcTurn(55);
            enc.steeringDrive(15, false, true);
            enc.steeringDrive(-23, false, false);
            enc.steeringDrive(23, false, true);


        }

        //     position 1 and 4
        if ((posit >= 140 && posit <= 190)) {

            mech.ejectStone();
            mech.clawRelease();
            enc.steeringDrive(22, false, false);
            mech.stopStone();
            enc.arcTurn(91);
            enc.steeringDrive(5, false, false);
            waitFor(100);
            mech.intakeStone();
            enc.steeringDrive(19, false, true);
            waitFor(125);
            enc.steeringDrive(6, false, false);
            mech.clawGrab();
            mech.stopStone();
            enc.arcTurn(-2);
            enc.steeringDrive(-18, false, true);
            enc.steeringDrive(-91, false, false);
            enc.arcTurn(90);
            enc.steeringDrive(-4, false, true);
            enc.steeringDrive(-9.5, false, false);
            mech.shoulderUp();
            waitFor(1500);
            mech.autoClaw();
            mech.shoulderDown();
            mech.foundationRelease();
            waitFor(1500);
            enc.steeringDrive(28, false, false);
            enc.arcTurn(-90);
            mech.foundationGrab();
            enc.arcTurn(55);
            enc.steeringDrive(14, false, true);
            enc.steeringDrive(-21, false, false);
            enc.steeringDrive(23, false, true);



        }
//                   position 2 and 5
        if ((posit >= 50 && posit <= 90)) {

            mech.ejectStone();
            mech.clawRelease();
            enc.steeringDrive(22, false, false);
            mech.stopStone();
            enc.arcTurn(91);
            enc.steeringDrive(9.5, false, false);
//            waitFor(100);
            mech.intakeStone();
            enc.steeringDrive(19.5, false, true);
            waitFor(125);
            enc.steeringDrive(8, false, false);
            mech.clawGrab();
            mech.stopStone();
            enc.arcTurn(-3);
//            waitFor(250);
            enc.steeringDrive(-18, false, true);
            enc.steeringDrive(-93, false, false);
            enc.arcTurn(90);
            enc.steeringDrive(-8, false, true);
            enc.steeringDrive(-10, false, false);
            mech.shoulderUp();
            waitFor(1000);
            mech.autoClaw();
            mech.shoulderDown();
            mech.foundationRelease();
            waitFor(1000);
            enc.steeringDrive(28, false, false);
            enc.arcTurn(-90);
            waitFor(50);
            mech.foundationGrab();
            enc.arcTurn(55);
            enc.steeringDrive(13, false, true);
            enc.steeringDrive(-21, false, false);
            enc.steeringDrive(23, false, true);


        }
    }
}









