package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Subsystems.Driving.RobotComponents;
import org.firstinspires.ftc.teamcode.Subsystems.Driving.SeansEncLibrary;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.Subsystems.Robot.RISELinearOpMode;
import org.firstinspires.ftc.teamcode.Subsystems.*;
import org.firstinspires.ftc.teamcode.Subsystems.VisionTargeting.SkystoneDetectionPhone;

@Autonomous(name = "Blue Depot",group = "Blue Auto" )
public class Blue_Auto_Depot_V2 extends LinearOpMode{

    SeansEncLibrary enc;
    SkystoneDetectionPhone cam;
    RobotComponents mech;

    public boolean isSkystone;

    public void runOpMode() throws InterruptedException {

        enc = new SeansEncLibrary(hardwareMap, telemetry, this);
        mech = new RobotComponents(hardwareMap, telemetry);
        cam = new SkystoneDetectionPhone(hardwareMap, telemetry);

        enc.init();
        mech.init();
        cam.init();

        while (true){

            isSkystone = cam.TFdetect(isSkystone);

            if(isStarted()){
                break;
            }

        }

        enc.steeringDrive(6, true,false);

        enc.gyroTurn(enc.TURN_SPEED,-90);

        enc.steeringDrive(12, true, false);

        enc.gyroTurn(enc.TURN_SPEED,0);

        enc.steeringDrive(38, true, false);

        enc.steeringDrive(-6,true,false);

        enc.gyroTurn(enc.TURN_SPEED, 90);

    }
}
