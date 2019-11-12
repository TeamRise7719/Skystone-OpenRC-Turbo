package org.firstinspires.ftc.teamcode.Autonomous;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;


import org.firstinspires.ftc.teamcode.SeansSpace.Subsystems.AutonomousPathing;
import org.firstinspires.ftc.teamcode.Subsystems.Driving.RobotComponents;
import org.firstinspires.ftc.teamcode.Subsystems.Driving.SeansEncLibrary;
import org.firstinspires.ftc.teamcode.Subsystems.Driving.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystems.Transitioning.AutoTransitioner;
import org.firstinspires.ftc.teamcode.Subsystems.VisionTargeting.SkystoneDetectionPhone;
import org.firstinspires.ftc.teamcode.Subsystems.VisionTargeting.SkystoneDetectionWebcam;

@Autonomous (name = "Red_Auto_V1", group = "Autonomous")
public class Red_Auto_V1 extends LinearOpMode {
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


        //Position 1 (4 from wall)
        enc.steeringDrive( 48, false);
        enc.steeringDrive(-24,false);
        //strafing here
        enc.gyroTurn(enc.TURN_SPEED, -90);
        mech.intakeStone();
        enc.gyroTurn(enc.TURN_SPEED, 180);
        enc.steeringDrive(80, false);

        //sample
        //vision code goes here
        mech.intakeStone();
        enc.steeringDrive(-12,  false);
        enc.gyroTurn(enc.TURN_SPEED, 90);
        enc.steeringDrive(80, false);
    }
}
