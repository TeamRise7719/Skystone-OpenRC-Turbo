package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.CRServo;


import org.firstinspires.ftc.teamcode.Subsystems.Driving.teleop.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystems.Driving.teleop.mecanumDrivetrain;
import org.firstinspires.ftc.teamcode.Subsystems.Driving.autonomous.encoderLibrary;
import org.firstinspires.ftc.teamcode.Subsystems.Sensing.visionLibrary;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Blue_Auto_V1", group = "Rise")
public class Blue_Auto_V1 extends LinearOpMode {

    private Drivetrain robot;
    int position;
    encoderLibrary enc;
    visionLibrary vis;

    @Override
    public void runOpMode() throws InterruptedException {

        enc = new encoderLibrary(hardwareMap, telemetry, this);


        //moves toward skyblock
        enc.gyroDrive(0.5,32, 0,false);
        //move to toward bridge
        enc.gyroDrive(0.5,-18,0,false);
        enc.gyroHold(0.5,45,2.5);
        enc.gyroDrive(0.5,27,0,false);
    }


}



















