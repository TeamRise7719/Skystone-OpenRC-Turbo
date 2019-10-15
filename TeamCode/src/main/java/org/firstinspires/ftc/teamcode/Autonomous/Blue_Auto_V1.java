package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;


import org.firstinspires.ftc.teamcode.Subsystems.Driving.ComponentClass;
import org.firstinspires.ftc.teamcode.Subsystems.Driving.SeansEncLibrary;
import org.firstinspires.ftc.teamcode.Subsystems.Driving.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystems.Transitioning.AutoTransitioner;
import org.firstinspires.ftc.teamcode.Subsystems.VisionTargeting.SkystoneDetectionPhone;
import org.firstinspires.ftc.teamcode.Subsystems.VisionTargeting.SkystoneDetectionWebcam;

@Autonomous(name = "Blue_Auto_V1", group = "Autonomous")
public class Blue_Auto_V1 extends LinearOpMode {

    private Drivetrain robot;
    private ComponentClass part;
    int position;
    SeansEncLibrary enc;
    SkystoneDetectionPhone phone;
    SkystoneDetectionWebcam webcam;
    DcMotor intake;


    @Override
    public void runOpMode() throws InterruptedException {

        enc = new SeansEncLibrary(hardwareMap, telemetry, this);
        part = new ComponentClass(hardwareMap,telemetry);


        //moves toward skyblock
        enc.gyroDrive(0.5,32, 0,false);
        //vision code goes here
        //intakes block
        part.intakeStone(1.0);
        sleep(1900);
        //move to toward bridge
        enc.gyroDrive(0.5,-18,0,false);
        enc.gyroHold(0.5,45,2.5);
        enc.gyroDrive(0.5,27,0,false);



        AutoTransitioner.transitionOnStop(this, "BasicTeleOp");//Transition to TeleOp
    }


}



















