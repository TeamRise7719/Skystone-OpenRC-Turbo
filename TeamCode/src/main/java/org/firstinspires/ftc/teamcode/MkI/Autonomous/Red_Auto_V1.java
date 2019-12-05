package org.firstinspires.ftc.teamcode.MkI.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MkI.Subsystems.Driving.RobotComponents;
import org.firstinspires.ftc.teamcode.MkI.Subsystems.Driving.SeansEncLibrary;
import org.firstinspires.ftc.teamcode.MkI.Subsystems.Transitioning.AutoTransitioner;

@Autonomous (name = "Red Auto V1", group = "MkI Red Auto")
public class Red_Auto_V1 extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        //int position;
        SeansEncLibrary enc = new SeansEncLibrary(hardwareMap, telemetry, this);
        enc.init();
        RobotComponents mech = new RobotComponents(hardwareMap, telemetry);

        waitForStart();


        //Position 1 (4 from wall)
        enc.steeringDrive( 48, false,false);
        enc.steeringDrive(-24,false,false);
        //strafing here
        enc.gyroTurn(enc.TURN_SPEED, -90);
        mech.intakeStone();
        enc.gyroTurn(enc.TURN_SPEED, 180);
        enc.steeringDrive(80, false,false);

        //sample
        //vision code goes here
        mech.intakeStone();
        enc.steeringDrive(-12,  false,false);
        enc.gyroTurn(enc.TURN_SPEED, 90);
        enc.steeringDrive(80, false,false);

        AutoTransitioner.transitionOnStop(this,"MkITeleOp");
    }
}
