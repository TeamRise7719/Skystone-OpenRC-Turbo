package org.firstinspires.ftc.teamcode.MkI.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MkI.Subsystems.Driving.RobotComponents;
import org.firstinspires.ftc.teamcode.MkI.Subsystems.Driving.SeansEncLibrary;
import org.firstinspires.ftc.teamcode.MkI.Subsystems.Transitioning.AutoTransitioner;

@Autonomous (name = "Blue Auto V2", group = "MkI Blue Auto")
public class Blue_Auto_V2 extends LinearOpMode {

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
        RobotComponents bacon = new RobotComponents(hardwareMap);

        enc.init();

        waitForStart();

        enc.steeringDrive(-28, false,false);
        enc.steeringDrive(6, true, true);
        bacon.returnPosition();
        waitFor(2000);
        enc.steeringDrive(28,true,false);
        bacon.outPosition();
        enc.steeringDrive(-24,true,true);
        enc.steeringDrive(-24,true,false);
        enc.gyroTurn(enc.TURN_SPEED,-90);
        enc.steeringDrive(24, true, false);

        AutoTransitioner.transitionOnStop(this,"MkITeleOp");
    }
}


