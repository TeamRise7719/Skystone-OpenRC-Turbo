package org.firstinspires.ftc.teamcode.MkI.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


import org.firstinspires.ftc.teamcode.SeansSpace.SeansSubsystems.Driving.AutonomousPathing;
import org.firstinspires.ftc.teamcode.MkI.Subsystems.Driving.RobotComponents;
import org.firstinspires.ftc.teamcode.MkI.Subsystems.Driving.SeansEncLibrary;
import org.firstinspires.ftc.teamcode.MkI.Subsystems.Driving.Drivetrain;
import org.firstinspires.ftc.teamcode.MkI.Subsystems.Transitioning.AutoTransitioner;
import org.firstinspires.ftc.teamcode.MkI.Subsystems.VisionTargeting.SkystoneDetectionPhone;

@Autonomous(name = "Blue Auto V1", group = "MkI Blue Auto")
public class Blue_Auto_V1 extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        //int position;
        SeansEncLibrary enc = new SeansEncLibrary(hardwareMap, telemetry, this);
        RobotComponents mech = new RobotComponents(hardwareMap);

        enc.init();

        waitForStart();

        //Position 1 (4 from wall)
        enc.steeringDrive( 48, false, false);
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


// other side

        //24
        //turn 120
        // drive 40
        //turn 45
        //drive 32
        //vision
        //intake
        //back 32
        //turn 120
        //drive 45
        AutoTransitioner.transitionOnStop(this,"MkITeleOp");    }


}