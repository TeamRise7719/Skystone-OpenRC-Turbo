package org.firstinspires.ftc.teamcode.Autonomous;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


import org.firstinspires.ftc.teamcode.SeansSpace.Subsystems.Driving.AutonomousPathing;
import org.firstinspires.ftc.teamcode.Subsystems.Driving.RobotComponents;
import org.firstinspires.ftc.teamcode.Subsystems.Driving.SeansEncLibrary;
import org.firstinspires.ftc.teamcode.Subsystems.Driving.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystems.VisionTargeting.SkystoneDetectionPhone;

@Autonomous (name = "Blue_Auto_V2", group = "Autonomous")
public class Blue_Auto_V2 extends LinearOpMode {


    private Drivetrain robot;
    boolean isSkystone;
    private RobotComponents mech;
    private AutonomousPathing path;
    //int position;
    SeansEncLibrary enc;
    SkystoneDetectionPhone phone;


    @Override
    public void runOpMode() throws InterruptedException {

        enc = new SeansEncLibrary(hardwareMap, telemetry, this);
        enc.init();
        mech = new RobotComponents(hardwareMap, telemetry);

        waitForStart();


         enc.steeringDrive(24, true);
         enc.gyroTurn(enc.TURN_SPEED, 90);
         enc.steeringDrive(48, true);
//        enc.gyroTurn(enc.TURN_SPEED, -90);
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

    }
}


