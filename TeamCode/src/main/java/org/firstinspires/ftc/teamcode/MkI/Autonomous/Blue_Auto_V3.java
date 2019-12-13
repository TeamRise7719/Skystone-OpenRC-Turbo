package org.firstinspires.ftc.teamcode.MkI.Autonomous;



import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.MkI.TeleOp.MkITeleOp;
import org.firstinspires.ftc.teamcode.SeansSpace.DogeCV.GGOpenCV;
import org.firstinspires.ftc.teamcode.SeansSpace.DogeCV.GGSkystoneDetector;
import org.firstinspires.ftc.teamcode.SeansSpace.DogeCV.VisionSystem;
import org.firstinspires.ftc.teamcode.SeansSpace.SeansSubsystems.Driving.AutonomousPathing;
import org.firstinspires.ftc.teamcode.MkI.Subsystems.Driving.RobotComponents;
import org.firstinspires.ftc.teamcode.MkI.Subsystems.Driving.SeansEncLibrary;
import org.firstinspires.ftc.teamcode.MkI.Subsystems.Driving.Drivetrain;
import org.firstinspires.ftc.teamcode.MkI.Subsystems.Transitioning.AutoTransitioner;
import org.firstinspires.ftc.teamcode.MkI.Subsystems.VisionTargeting.SkystoneDetectionPhone;


@Autonomous (name = "Blue Auto Both", group = "Mlk Blue Auto")
public class Blue_Auto_V3 extends LinearOpMode {

    GGSkystoneDetector vis;
    GGOpenCV detector;
    ElapsedTime etime = new ElapsedTime();
    double posit = 0;

    public void waitFor(int time){
        time = time/1000;
        etime.reset();
        while ((etime.time() < time)&&(opModeIsActive())) {
            idle();
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {

        SeansEncLibrary enc = new SeansEncLibrary(hardwareMap, telemetry, this);
        RobotComponents mech = new RobotComponents(hardwareMap);
        GGOpenCV detector = new GGOpenCV(GGOpenCV.Cam.PHONE, hardwareMap);
        detector.startCamera();
        detector.startLook(VisionSystem.TargetType.SKYSTONE);

        enc.init();



        while (!this.isStarted){
            if (detector.found()){
                telemetry.addData("SS Found!", "");
                telemetry.addData("X: ", detector.detector.foundRectangle().x);
                telemetry.addData("Y: ", detector.detector.foundRectangle().y);

                posit = detector.detector.foundRectangle().x;
            } else {
                telemetry.addData("SS not found.", "");
            }

            telemetry.update();
        }

        detector.stopLook();


        if((posit>=260&&posit<=280)||(posit>=40&&posit<60)){

            enc.gyroTurn(enc.TURN_SPEED,-90);
            enc.steeringDrive(-10,false,true);
            enc.steeringDrive(44.3,false,false);
            enc.steeringDrive(4, false, false);
            waitFor(1000);
            mech.intakeStone();
            waitFor(500);
            enc.steeringDrive(-18,false,true);
            enc.steeringDrive(-33,false,false);
            waitFor(1000);
            mech.ejectStone();



        }


        if ((posit>=190&&posit<=210)||(posit>=6&&posit<=10)){

            enc.gyroTurn(enc.TURN_SPEED,-90);
            enc.steeringDrive(-20,false,true);
            enc.steeringDrive(44.3,false,false);
            enc.steeringDrive(4, false, false);
            waitFor(1000);
            mech.intakeStone();
            waitFor(500);
            enc.steeringDrive(-18,false,true);
            enc.steeringDrive(-43,false,false);
            waitFor(1000);
            mech.ejectStone();


        }

        if ((posit>=124&&posit<=130)||(posit==0)){

            enc.gyroTurn(enc.TURN_SPEED,-90);
            enc.steeringDrive(-30,false,true);
            enc.steeringDrive(44.3,false,false);
            enc.steeringDrive(4, false, false);
            waitFor(1000);
            mech.intakeStone();
            waitFor(500);
            enc.steeringDrive(-18,false,true);
            enc.steeringDrive(-53,false,false);
            waitFor(1000);
            mech.ejectStone();


        }
    }










}
