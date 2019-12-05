package org.firstinspires.ftc.teamcode.MkI.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MkI.Subsystems.Driving.SeansEncLibrary;
import org.firstinspires.ftc.teamcode.MkI.Subsystems.Transitioning.AutoTransitioner;

@Autonomous (name = "Red Auto V2" , group = "MkI Red Auto" )
public class Red_Auto_V2 extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        //int position;
        SeansEncLibrary enc = new SeansEncLibrary(hardwareMap, telemetry, this);
        enc.init();

        waitForStart();


        enc.steeringDrive(64, true,false);
        enc.gyroTurn(enc.TURN_SPEED, -90);
        enc.steeringDrive(32, true,false);
        enc.steeringDrive(55,true,false);
//        enc.gyroTurn(enc.TURN_SPEED, 90);
//        enc.steeringDrive(40, false);
//        enc.steeringDrive(-24, false);
//        //strafing here
//        enc.gyroTurn(enc.TURN_SPEED, -90);
//        mech.intakeStone();
//        enc.gyroTurn(enc.TURN_SPEED, 180);
//        enc.steeringDrive(80, false);
//
//        //sample
//        //vision code goes here
//        mech.intakeStone();
//        enc.steeringDrive(-12, false);
//        enc.gyroTurn(enc.TURN_SPEED, 90);
//        enc.steeringDrive(80, false);

        AutoTransitioner.transitionOnStop(this,"MkITeleOp");
    }
}




