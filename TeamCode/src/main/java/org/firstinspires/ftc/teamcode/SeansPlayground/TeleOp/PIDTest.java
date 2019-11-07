package org.firstinspires.ftc.teamcode.SeansPlayground.TeleOp;

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

                enc.gyroDrive(enc.TURN_SPEED,24.5,0, false);
            }
            if (gamepad1.y) {
                enc.gyroTurn(enc.DRIVE_SPEED,90);
            }
            if (gamepad1.b) {
                enc.gyroHold(enc.TURN_SPEED,180,30);
            }
        }
    }
}