package org.firstinspires.ftc.teamcode.Fraser.Autonomous.JordanAutonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Fraser.Subsystems.Drive.FraserComponents;
import org.firstinspires.ftc.teamcode.Fraser.Subsystems.Drive.SeansEncLibrary;
import org.firstinspires.ftc.teamcode.Fraser.Subsystems.Transitioning.AutoTransitioner;


@Autonomous (name = "Blue Auto Stones", group = "Fraser Blue Auto")
public class Blue_Auto_Stones extends LinearOpMode {

    ElapsedTime etime = new ElapsedTime();

    public void waitFor(int time){
        time = time/1000;
        etime.reset();
        while ((etime.time() < time)&&(opModeIsActive())) {
            idle();
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {

        //int position;
        SeansEncLibrary enc = new SeansEncLibrary(hardwareMap, telemetry, this);
        FraserComponents mech = new FraserComponents(hardwareMap);

        enc.init();

        waitForStart();

        //Position 1 (4 from wall);
        //Todo: add vision
        enc.steeringDrive( -44.3, false, true);
        mech.intake.setPower(-1);
        waitFor(1000);
        mech.ejectStone();
        waitFor(500);
        enc.steeringDrive(4, false, false);
        mech.stopStone();
        enc.steeringDrive(18,false,true);
        enc.steeringDrive(-33,false,false);
        enc.gyroTurn(enc.TURN_SPEED,-90);
        mech.intakeStone();
        waitFor(1000);
        enc.gyroTurn(enc.TURN_SPEED,0);
        enc.steeringDrive(35,false,false);
        enc.steeringDrive(-18,false,true);
        mech.ejectStone();
        waitFor(1000);
        enc.steeringDrive(8, false, false);
        mech.stopStone();
        enc.steeringDrive(18,false,true);
        enc.steeringDrive(-50,false,false);
        enc.gyroTurn(enc.TURN_SPEED,-90);
        mech.intakeStone();
        waitFor(1000);
        enc.gyroTurn(enc.TURN_SPEED,0);
        enc.steeringDrive(20,false,false);


        AutoTransitioner.transitionOnStop(this, "Fraser TeleOp");
    }

}
