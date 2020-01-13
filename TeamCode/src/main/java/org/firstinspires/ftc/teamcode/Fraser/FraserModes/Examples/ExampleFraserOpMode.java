package org.firstinspires.ftc.teamcode.Fraser.FraserModes.Examples;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Fraser.FraserModes.FraserOpMode;

/**
 * Created by Sean Cardosi on 2020-01-11.
 */
@TeleOp(name = "Example FraserOpMode", group = "FraserModes")
public class ExampleFraserOpMode extends FraserOpMode {

    @Override
    public void init() {}

    @Override
    public void init_loop() {}

    @Override
    public void start() {}

    @Override
    public void loop() {
       robot.drive(gamepad1,telemetry);
    }

    @Override
    public void stop() { }
}
