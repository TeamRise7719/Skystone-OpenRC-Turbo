package org.firstinspires.ftc.teamcode.SeansPlayground.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.SeansPlayground.PIDTuning.SeansPIDTuningLibrary;

@TeleOp(name = "TestPID",group = "Testing" )
public class PIDTest extends LinearOpMode {

//    SeansEncLibrary enc;
    SeansPIDTuningLibrary PID;

    public void runOpMode() throws InterruptedException {

//        enc = new SeansEncLibrary(hardwareMap, telemetry, this);
//        enc.init();
        PID = new SeansPIDTuningLibrary(hardwareMap, telemetry, this);
        PID.init();

        waitForStart();

        while (opModeIsActive()) {

            PID.getPID();

            if (gamepad1.x) {

                PID.gyroDrive(0.75,24.5,0, false);
            }
            if (gamepad1.y) {
                PID.gyroTurn(0.75,90);
            }
        }
        PID.shutdown();
    }
}