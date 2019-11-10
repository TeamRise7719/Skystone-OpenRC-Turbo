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

                enc.steeringDrive(24, false);
            }
            if (gamepad1.y) {
                enc.steeringDrive(-12, false);
            }
            if (gamepad1.b) {
                enc.gyroTurn(enc.TURN_SPEED, 90);
            }
            if (gamepad2.b) {//GAMEPAD 2 Strafe left
                enc.steeringStrafe(24, enc.LEFT, true);
            }
            if (gamepad2.x) {//GAMEPAD 2 Strafe right
                enc.steeringStrafe(24,enc.RIGHT, true);
            }


        }
    }
}