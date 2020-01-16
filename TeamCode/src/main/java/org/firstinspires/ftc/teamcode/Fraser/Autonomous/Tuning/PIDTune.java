package org.firstinspires.ftc.teamcode.Fraser.Autonomous.Tuning;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Fraser.Subsystems.Drive.FraserComponents;
import org.firstinspires.ftc.teamcode.Fraser.Subsystems.Drive.SeansEncLibrary;

@Autonomous(name = "PID Test", group = "Tuning")
public class PIDTune extends LinearOpMode {

    private ElapsedTime etime = new ElapsedTime();

    final boolean steeringToggle = true;
    final boolean driveTest = false;
    final boolean turnTest = true;
    final boolean viewTelemetry = true;
    final int msBetweenMovements = 1000;

    @Override
    public void runOpMode() {

        SeansEncLibrary enc = new SeansEncLibrary(hardwareMap, telemetry, this);
        enc.init();

        waitForStart();

        if (turnTest) {
            enc.gyroTurn(1, 90);
            sleep(msBetweenMovements);
        }
        if (driveTest) {
            telemetry.addLine("Forward");
            telemetry.addLine("Toggle: " + steeringToggle);
            telemetry.update();
            enc.steeringDrive(24, steeringToggle, false);
            waitFor(msBetweenMovements);
            telemetry.addLine("Backward");
            telemetry.addLine("Toggle: " + steeringToggle);
            telemetry.update();
            enc.steeringDrive(-24, steeringToggle, false);
            waitFor(msBetweenMovements);
            telemetry.addLine("Left");
            telemetry.addLine("Toggle: " + steeringToggle);
            telemetry.update();
            enc.steeringDrive(-24, steeringToggle, true);
            waitFor(msBetweenMovements);
            telemetry.addLine("Right");
            telemetry.addLine("Toggle: " + steeringToggle);
            telemetry.update();
            enc.steeringDrive(24, steeringToggle, true);
            waitFor(msBetweenMovements);
        }
        if (viewTelemetry) {
            while (opModeIsActive()) {

            }
        }
    }
    public void waitFor(int time) {
        time = time / 1000;
        etime.reset();
        while ((etime.time() < time) && (opModeIsActive())) {
            idle();
        }
    }
}
