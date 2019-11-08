package org.firstinspires.ftc.teamcode.SeansPlayground.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.Driving.SeansEncLibrary;
import org.firstinspires.ftc.teamcode.Subsystems.Robot.RISELinearOpMode;

@TeleOp(name = "TestPID",group = "Testing" )
public class PIDTest extends RISELinearOpMode {

    SeansEncLibrary enc;
    public void runOpMode() throws InterruptedException {

        enc = new SeansEncLibrary(hardwareMap, telemetry, this);
        enc.init();

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.x) {

                enc.steeringDrive(18, true);
            }
            if (gamepad1.y) {
                enc.steeringDrive(-12, true);
            }
            if (gamepad1.b) {
                enc.gyroTurn(enc.TURN_SPEED, 90);
            }

            telemetry.addData("Active", "");
            telemetry.update();
        }
    }
}