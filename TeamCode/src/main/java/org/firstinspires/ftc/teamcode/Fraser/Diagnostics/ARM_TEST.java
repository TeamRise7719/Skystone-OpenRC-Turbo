package org.firstinspires.ftc.teamcode.Fraser.Diagnostics;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Fraser.Subsystems.Drive.FraserComponents;
import org.firstinspires.ftc.teamcode.Fraser.Subsystems.Drive.SeansEncLibrary;

@Autonomous (name = "ARM TEST" ,  group = " Fraser Auto")
public class ARM_TEST extends LinearOpMode {

    ElapsedTime etime = new ElapsedTime();
    private double posit = 0;

    public void waitFor(int time) {
        time = time / 1000;
        etime.reset();
        while ((etime.time() < time) && (opModeIsActive())) {
            idle();
        }
    }

    @Override
    public void runOpMode() {

        SeansEncLibrary enc = new SeansEncLibrary(hardwareMap, telemetry, this);
        FraserComponents mech = new FraserComponents(hardwareMap);

        enc.init();
        mech.init();


        /** Arm Test
         *
         */
            mech.clawRelease();
            waitFor(1000);
            mech.autoIntake();
            waitFor(1000);
            mech.clawGrab();
            waitFor(1000);
            mech.shoulderUp();
            waitFor(1500);
            mech.clawRelease();
            waitFor(1000);
            mech.clawGrab();
            waitFor(1000);
            mech.shoulderDown();
            waitFor(1500);

    }
}
