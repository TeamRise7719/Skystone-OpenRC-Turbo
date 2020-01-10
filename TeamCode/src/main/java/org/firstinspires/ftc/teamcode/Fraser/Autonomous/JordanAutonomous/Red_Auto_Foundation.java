package org.firstinspires.ftc.teamcode.Fraser.Autonomous.JordanAutonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Fraser.Subsystems.Drive.FraserComponents;
import org.firstinspires.ftc.teamcode.Fraser.Subsystems.Drive.SeansEncLibrary;
import org.firstinspires.ftc.teamcode.Fraser.Subsystems.Transitioning.AutoTransitioner;


@Autonomous (name = "Red Auto Foundation" , group = "Fraser Red Auto" )
public class Red_Auto_Foundation extends LinearOpMode {
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
        FraserComponents bacon = new FraserComponents(hardwareMap);

        enc.init();

        waitForStart();

        enc.steeringDrive(-20, false,false);
        enc.steeringDrive(-6.5, false,false);
        enc.steeringDrive(11.5, false, true);
        bacon.foundationRelease();
        waitFor(2000);
        enc.steeringDrive(28,false,false);
        bacon.foundationGrab();
        enc.steeringDrive(-30,false,true);

        enc.steeringDrive(-22,false,false);
        enc.gyroTurn(enc.TURN_SPEED,-90);
        enc.steeringDrive(24, false, false);



        AutoTransitioner.transitionOnStop(this,"Fraser TeleOp");
    }
}





