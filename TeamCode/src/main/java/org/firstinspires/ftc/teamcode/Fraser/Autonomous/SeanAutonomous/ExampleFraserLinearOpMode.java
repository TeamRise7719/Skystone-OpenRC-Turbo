package org.firstinspires.ftc.teamcode.Fraser.Autonomous.SeanAutonomous;

import org.firstinspires.ftc.teamcode.Fraser.FraserModes.FraserLinearOpMode;

/**
 * Created by Sean Cardosi on 2020-01-09.
 *
 * A custom LinearOpMode to make coding easier!
 * This is how I think this all should work. I'm probably wrong.
 */
public class ExampleFraserLinearOpMode extends FraserLinearOpMode {

    double position = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        /*
        You can also replace this with skystoneScanProcess(position) if you want to scan Skystones.
         */
        waitForStart();

        /*
        waitFor() is built-in. No need to declare it yourself.
         */
        waitFor(4000);

        /*
        AutoTransitioner is also built-in! It will transition to Fraser TeleOp.
         */
        toTeleOp();
    }
}
