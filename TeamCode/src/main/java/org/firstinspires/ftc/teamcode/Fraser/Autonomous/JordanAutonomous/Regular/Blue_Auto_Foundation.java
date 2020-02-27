package org.firstinspires.ftc.teamcode.Fraser.Autonomous.JordanAutonomous.Regular;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Fraser.Subsystems.Drive.FraserComponents;
import org.firstinspires.ftc.teamcode.Fraser.Subsystems.Drive.SeansEncLibrary;
import org.firstinspires.ftc.teamcode.Fraser.Subsystems.Transitioning.AutoTransitioner;

@Autonomous (name = "Blue Auto Foundation", group = "Fraser Blue Auto")
public class Blue_Auto_Foundation extends LinearOpMode{

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
        bacon.init();

        waitForStart();

        enc.steeringDrive(-30, false,false);
        enc.steeringDrive(11.5, false, true);
        bacon.foundationRelease();
        waitFor(2000);

        enc.steeringDrive(40
                ,false,false);
        bacon.foundationGrab();
        enc.steeringDrive(-49,false,true);

        AutoTransitioner.transitionOnStop(this,"Fraser TeleOp");
    }
}


