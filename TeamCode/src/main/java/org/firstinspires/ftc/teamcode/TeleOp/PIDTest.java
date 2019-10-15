package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.Driving.SeansEncLibrary;
@TeleOp(name = "TestPID",group = "Testing" )
public class PIDTest extends LinearOpMode {

    SeansEncLibrary enc;

    public void runOpMode() throws InterruptedException {

        enc = new SeansEncLibrary(hardwareMap, telemetry, this);
        enc.init();


        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.x) {

                enc.gyroDrive(0.1,24.5,0, false);
            }
            if (gamepad1.y) {
                enc.gyroTurn(0.1,90);
            }
        }
    }
}