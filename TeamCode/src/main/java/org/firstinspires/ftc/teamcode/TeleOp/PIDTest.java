package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.Driving.SeansEncLibrary;
import org.firstinspires.ftc.teamcode.Subsystems.Robot.RISELinearOpMode;

@TeleOp(name = "TestPID",group = "Testing" )
public class PIDTest extends LinearOpMode {

    SeansEncLibrary enc;
    public void runOpMode() throws InterruptedException {

        enc = new SeansEncLibrary(hardwareMap, telemetry, this);
        enc.init();

        waitForStart();

        //As long as the opMode is on, the loop will not break
        while (opModeIsActive()) {

            if (gamepad1.x) {

                enc.steeringDrive(60,true,false,1);//24 true
            }
            if (gamepad1.y) {
                enc.steeringDrive(12, false,true,1);
            }
            if (gamepad1.b) {
                enc.gyroTurn(enc.TURN_SPEED, 90);
            }
            if (gamepad2.b) {
            }
            if (gamepad2.x) {
            }


        }
    }
}